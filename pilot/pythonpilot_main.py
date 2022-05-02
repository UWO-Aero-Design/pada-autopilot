
#!/usr/bin/python
'''
*****************************************************************************
Copyright (C) 2021 by Robotics In Flight, LLC. (roboticsinflight.com)
David Haessig & Michael Vogas

PythonPilot is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

PythonPilot is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program. If not, see https://www.gnu.org/licenses.

PythonPilot consists of the main routine -- PythonPilot_main_{yymmdd}.py --
and subroutines contained in the following files:
-- gnc.py guidance, navigation and control functions
-- flt_st.py flight state machine functions
-- esc.py PWM control generation function
-- imu.py IMU read function
-- kb_cmd.py keyboard read function

PythonPilot is a trademark of Robotics In Flight, LLC.
*****************************************************************************
'''

from __future__ import division
import socket
import string
import fcntl
import getopt
import math
import smbus
import datetime
import shutil
import pdb
import sys
from RPIO import PWM
import RPIO
import ctypes
from ctypes.util import find_library
import time
import os
import signal

# calls to modules subdirectory for module inclusion in main
from modules.esc import ESC
from modules.kb_cmd import keyPressed
from modules.imu import MPU6050
from modules.flt_st import statemachine
from modules.rif_cmd import rccmd
from modules.rif_cmd import rccmd_init
from modules.rif_cmd import rc_cal
from modules.rif_cmd import heading_cal
from modules.gnc import guidance
from modules.gnc import nav
from modules.gnc import control
from modules.gnc import setpoint
from modules.gnc import gnc_init
from modules.gnc import imu_cal

# *****************************************************************************************
# Function Definitions
# *****************************************************************************************

# ---------------------------------------------------------------------
# RPIO Pulse Width Modulation (PWM) functions
# ---------------------------------------------------------------------
def RpioSetup():
# setup to use the Broadcomm (BCM)SOC channel pin numbers
RPIO.setmode(RPIO.BCM)

def ShutdownRPIO():
PWM.cleanup()
RPIO.cleanup()
time.sleep(0.10)
# sys.exit(0)           # This was uncommented previously

def resizeHandler(signum, frame):
    print("resize-window signal caught")

    # ---------------------------------------------------------------------
    # Invoke RTpreempt for real time operation of the python code
    # with priority 99 in Interpreter Mode. Select one of the 2.
    # ---------------------------------------------------------------------
    os.system("sudo chrt -f -p 99 `pgrep python` # -f : SCHED_FIFO, 1 : static priority 1")
    # ---------------------------------------------------------------------
    # Invoke RTpreempt for real time operation of the python code with
    # priority 99 in Compiled Mode. Select one of the 2.
    # ---------------------------------------------------------------------
    #os.system("sudo chrt -f -p 99 `pgrep PiQ_main` # -f : SCHED_FIFO, 1 : static priority 1")

    # Auto-start on bootup - set the following variable to True if this code is to be executed
    # at bootup without a console such as PuTTy. Otherwise if PuTTy is engaged, set 'auto_st' to False.
    auto_st = False #True

    # ---------------------------------------------------------------------
    # MPU6050 Initialization
    # ---------------------------------------------------------------------
    mpu6050 = MPU6050()
    mpu6050.Setup()
    C = mpu6050.ReadData()
    Cp = [0, 0, 0, 0, 0, 0, 0]
    mpu6050.Setup()

    # ---------------------------------------------------------------------
    # Time variables initialization
    # ---------------------------------------------------------------------
    prev_sample_time = time.time()

    # --------------------------------------------------------------------------------------
    # ESC & RPIO Initialization
    # --------------------------------------------------------------------------------------
    # Raspberry Pi connector pin numbers
    # The following are for the Alpha Units with 10x2 pin connector on the rPi
    ESC_BL = 18
    ESC_FL = 23
    ESC_FR = 22
    ESC_BR = 17
    # The following are for the Beta Units with the 20x2 pin connector on the rPi
    #ESC_BL = 13 # 18
    #ESC_FL = 5 # 23
    #ESC_FR = 18 # 22
    #ESC_BR = 21 # 17
    pin_list = [ESC_BR, ESC_BL, ESC_FR, ESC_FL]
    esc_list = []
    for esc_index in range(0, 4):
        esc = ESC(pin_list[esc_index])
        esc_list.append(esc)
    for esc in esc_list:
        esc.update(0)
    RpioSetup()
    signal.signal(signal.SIGWINCH, resizeHandler)

    #---------------------------------------------------------------------------
    # Prime the ESC's with a fast then slow prop spin for 500 msec
    #---------------------------------------------------------------------------
    delta_spin = 1000
    for esc in esc_list:
        esc.update(delta_spin)
        time.sleep(0.046)
        delta_spin = 0
    for esc in esc_list:
        esc.update(delta_spin)
        time.sleep(0.5)
        delta_spin = 100
    for esc in esc_list:
        esc.update(delta_spin)
        time.sleep(0.5)
        delta_spin = 0
    for esc in esc_list:
        esc.update(delta_spin)
    print("ESCs Primed: ")

    # ---------------------------------------------------------------------
    # Controller & Navigation Initialization
    # ---------------------------------------------------------------------
    # ESC control conversion coefficients
    d = 12.5/39.4 # moment arm (meter)
    Ct = 0.01 # Newtons thrust per prop per ESC count
    Ctd = 2*d*Ct # N-m torque per delta-count

    # ------------------------------------------------------------------------------------------
    # Flight State Initialization
    # ------------------------------------------------------------------------------------------
    flt_state = -1 # initial flight state - Preflight Calibration & Start-up State
    start_time = time.time()
    telem_flag = 0 # telemetry transfer index
    telm_idx = 0 # telemetry index
    pindx = 0 # telemetry screen writing index
    #cv = 0 # control variable flag set to 0 for roll/pitch angle, yaw rate, thrust (raw) control
    cv = 1 # control variable flag set to 1 for roll/pitch angle, integrated yaw rate, and altitude setpoint control

    # ------------------------------------------------------------------------------------------
    # Miscellaneous initializations
    # ------------------------------------------------------------------------------------------
    char = []
    key_cnt = 119
    # ------------------------------------------------------------------------------------------
    # Telemetry initializations
    # ------------------------------------------------------------------------------------------
    # pre-allocate array for flight telemetry data storage
    Ns = 8000 # number of sample values to store in data logging array
    Nv = 17 # number of data elements to store in data logging array
    Dc = 10 # decimation of telemetry data
    tel_save = True
    loop = -1
    dloop = Dc - 1
    temp = [float(0)]*Ns
    flt_dat = []
    for i in range(Nv):
        flt_dat.append(temp[:])
    temp = []

    # Static Variables
    statics_nav = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 0, 0, 0, 0, 0]
    statics_cntr = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
    statics_sp = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
    statics_sensor = [0, 0]
    statics_guid = [0, -5]
    statics_rc = [0, -1, 0]

    # Setpoint variable initialization
    thet_roll_sp = 0
    thet_pitch_sp = 0
    thet_yaw_sp = 0
    vthrust_sp = 0
    omeg_yaw_sp = 0
    alt_sp = 0

    # IMU sensed gyro y axis variance (sum and dump) variable
    sumGz = 0
    sumAx = 0
    sumAy = 0
    Nsum = 1

    # RC Stick Position initialization (relative to zero location)
    stk_r = 0 # stick 0 (roll)
    stk_p = 0 # stick 1 (pitch)
    stk_v = 0 # stick 2 (vertical)
    stk_y = 0 # stick 3 (yaw)
    ch_5 = 0 # switch ch 5
    ch_6 = 0 # switch ch 6
    ch_7 = 0 # knob ch 7
    ch_8 = 0 # switch ch 8

    # Setpoint zero location and other RC input initialization
    sp_v = 0
    sp_y = 0
    sp_r = 0
    sp_p = 0
    sw5 = 0
    sw6 = 0
    sp7 = 0
    sw8 = 0

    # GPS, baro altitude and magnetic heading variable initialization
    lat = 0
    lon = 0
    gps_alt = 0
    rx_alt = 0
    rx_head = 0
    magx = 0
    magy = 0
    magz = 0
    rx_altp = 0
    magxp = 0
    magyp = 0
    magzp = 0
    stk_rp = 0
    stk_pp = 0
    stk_vp = 0
    stk_yp = 0
    ch_5p = 0
    ch_6p = 0
    ch_7p = 0
    ch_8p = 0

    # initial control levels
    u_vert = 0
    u_pitch = 0
    u_roll = 0
    u_yaw = 0

    # motor drive off flags
    em_off = False

    print("\n\n********************************************************************************")
    print(" Welcome to PiQuad -- Reading Flight Data Files")
    print("********************************************************************************")

    # ---------------------------------------------------------------------
    # Read IMU cal data
    # ---------------------------------------------------------------------
    [cal_data, mag_cal] = gnc_init()

    # ---------------------------------------------------------------------
    # Spread spectrum receiver serial link initialization
    # ---------------------------------------------------------------------
    serial_port_index = "T" # string based on commanded output from the CRUIS
    prnt_flag = 0 # RC stick command print flag
    [rc_port, RC_cal_data, alt_zero_data, Ready] = rccmd_init(serial_port_index, prnt_flag)
    statics_sp = [RC_cal_data[0][0], RC_cal_data[0][0],RC_cal_data[0][0],RC_cal_data[0][0], \
    RC_cal_data[0][1], RC_cal_data[0][1],RC_cal_data[0][1],RC_cal_data[0][1], 0, 0, 0, 0, 0, 0, 0, 0]

    if Ready:
        print("\n\n********************************************************************************")
        print(" Pre-flight 45 second calibration in process ")
        print("********************************************************************************")

        # *****************************************************************************************
        # Main Loop
        # *****************************************************************************************
        while Ready:

            #-----------------------------------------------------------------------------------
            # IMU Read
            #-----------------------------------------------------------------------------------
            C = mpu6050.ReadData() # Rate and acceleration in deg/sec and G's respectively
            if C.Gz == 0: # catch an i2c IMU read error and set C to previously measured output
                C = Cp
            Cp = C
            # Apply a coordinate conversion for the Prototype units (Proto 2&3) ****************
            # changing from x-front, y-left, z-up to x-front, y-right, z-down
            #Ax = C.Gx
            #Ay = -C.Gy
            #Az = -C.Gz
            #Gx = C.Gyrox
            #Gy = -C.Gyroy
            #Gz = -C.Gyroz
            # Apply a coordinate conversion for the Alpha-Beta units, **************************
            # changing from x-left, y-front, z-down to x-front, y-right, z-down
            Ax = C.Gy
            Ay = -C.Gx
            Az = C.Gz
            Gx = C.Gyroy
            Gy = -C.Gyrox
            Gz = C.Gyroz
            #print ("IMU: %5.3f %5.3f %5.3f %5.3f %5.3f %5.3f " % (Ax, Ay, Az, Gx, Gy, Gz))
            #-----------------------------------------------------------------------------------
            # Compute time dependent variables
            #-----------------------------------------------------------------------------------
            current_time = time.time()
            delta_time = current_time - prev_sample_time
            on_time = current_time - start_time
            prev_sample_time = current_time

            # ----------------------------------------------------------------------------------
            # Keyboard inputs
            # ----------------------------------------------------------------------------------
            key_cnt += 1
            if key_cnt == 120: # Read keyboard at ~3 Hz if running with console (auto_st = False)
                key_cnt = 0
            if auto_st == False:
                char = keyPressed()
            else:
                char = []
            if char == False:
                char = []
            if char == 's':
                tel_save = False

            # ----------------------------------------------------------------------------------
            # Radio Controller inputs
            # ----------------------------------------------------------------------------------
            new_stick = 0
            if ((key_cnt-15)%30 == 0): # Read RC transmit at ~8 Hz
                new_stick = 1
                [stk_r, stk_p, stk_v, stk_y, rx_alt, magx, magy, magz, lat, lon, gps_alt, gps_head, ch_5, ch_6, ch_7, ch_8] = \
                rccmd(rc_port, serial_port_index, alt_zero_data, prnt_flag)
            # Catch an serial read glitch and set magnetometer values to previously measured outputs
            if magx == []:
                magx = magxp
                magy = magyp
                magz = magzp
            if stk_r == []:
                stk_r = stk_rp
            if stk_p == []:
                stk_p = stk_pp
            if stk_v == []:
                stk_v = stk_vp
            if stk_y == []:
                stk_y = stk_yp
            if ch_5 == []:
                ch_5 = ch_5p
            if ch_6 == []:
                ch_6 = ch_6p
            if ch_7 == []:
                ch_7 = ch_7p
            if ch_8 == []:
                ch_8 = ch_8p

            magxp = magx
            magyp = magy
            magzp = magz
            stk_rp = stk_r
            stk_pp = stk_p
            stk_vp = stk_v
            stk_yp = stk_y
            ch_5p = ch_5
            ch_6p = ch_6
            ch_7p = ch_7
            ch_8p = ch_8

            # Catch a baro-alt read glitch and set to previous baro measurement
            try:
                if rx_alt == [] or abs(rx_alt - rx_altp) > 10:
                    rx_alt = rx_altp
                else:
                    rx_altp = rx_alt
            except:
                rx_alt = rx_altp

            # ----------------------------------------------------------------------------------
            # Flight State Machine
            # ----------------------------------------------------------------------------------
            if new_stick:
                flt_st_p = flt_state;
                [flt_state, statics_rc, telem_flag, telm_idx, cv, RC_cal_data, em_off] = \
                statemachine(flt_state, on_time, char, sp_r, sp_p, sp_y, sp_v, sw5, sw6, sw8, umag, statics_rc, mpu6050, \
                rc_port, serial_port_index, cal_data, RC_cal_data, mag_cal, telem_flag, telm_idx, cv )
                char = []
            # Quick prop spin to indicate pre-flight cal completed
            if flt_state == 0 and flt_st_p == -1:
                delta_spin = 1000
            for esc in esc_list:
                esc.update(delta_spin)
                time.sleep(0.08)
                delta_spin = 0
            for esc in esc_list:
                esc.update(delta_spin)

            # ----------------------------------------------------------------------------------
            # Guidance, Navigation, Control functions
            # ----------------------------------------------------------------------------------
            if new_stick:
                # ******************************
                # Setpoint computation
                # ******************************
                [sp_r, sp_p, sp_y, sp_v, sw5, sw6, sp7, sw8, statics_sp] = \
                setpoint(cv, stk_r, stk_p, stk_y, stk_v, ch_5, ch_6, ch_7, ch_8, statics_sp, RC_cal_data)

                # ******************************
                # Guidance Computation
                # ******************************
                [thet_roll_sp, thet_pitch_sp, thet_yaw_sp, omeg_yaw_sp, vthrust_sp, alt_sp, statics_guid] = \
                guidance(cv, sp_r, sp_p, sp_y, sp_v, statics_guid, statics_nav[2], flt_state)

                # ******************************
                # Navigation
                # ******************************
                [omega_r, omega_p, omega_y, vx_kp1, vy_kp1, vz_kp1, statics_nav, statics_sensor] = \
                nav(Ax, Ay, Az, Gx, Gy, Gz, mpu6050, delta_time, statics_nav, cal_data, rx_alt, magx, magy, magz, mag_cal, statics_sensor, cv)
                xr_kp1 = statics_nav[0]
                xp_kp1 = statics_nav[1]
                xy_kp1 = statics_nav[2]
                pz_k = statics_nav[12]

                # ******************************
                # Control
                # ******************************
                [u_pitch, u_yaw, u_roll, u_vert, statics_cntr] = \
                control(cv, thet_pitch_sp, thet_roll_sp, omeg_yaw_sp, thet_yaw_sp, alt_sp, xr_kp1, xp_kp1, xy_kp1, \
                omega_r, omega_p, omega_y, vx_kp1, vy_kp1, vz_kp1, pz_k, statics_cntr)
                # Compute prop control level for use in avoiding unsafe flight mode change
                umag = max(vthrust_sp, (u_vert + (abs(u_pitch)+abs(u_roll)+abs(u_yaw))/Ctd) )

            #-----------------------------------------------------------------------------------
            # ESC Operation
            #-----------------------------------------------------------------------------------

            # Convert control torque (N-m) to a delta_ESC value in counts
            delta_ESC_p = u_pitch / Ctd
            delta_ESC_r = u_roll / Ctd
            delta_ESC_y = u_yaw / Ctd
            if cv == 0:
                delta_ESC_v = vthrust_sp
            elif cv == 1:
                delta_ESC_v = u_vert

            # Zero motor controls in all modes except Flight
            if ( em_off or flt_state<1 or flt_state>2 ):
                delta_ESC_v = 0
                delta_ESC_r = 0
                delta_ESC_p = 0
                delta_ESC_y = 0

            esc_index = 0
            for esc in esc_list:
                #---------------------------------------------------------------------------
                # Update blade speeds by linearly combining the 4 input command levels
                #---------------------------------------------------------------------------
                esc_index += 1
                if esc_index == 1:
                    delta_spin = -delta_ESC_p - delta_ESC_r - delta_ESC_y + delta_ESC_v
                if esc_index == 2:
                    delta_spin = -delta_ESC_p + delta_ESC_r + delta_ESC_y + delta_ESC_v
                if esc_index == 3:
                    delta_spin = delta_ESC_p - delta_ESC_r + delta_ESC_y + delta_ESC_v
                if esc_index == 4:
                    delta_spin = delta_ESC_p + delta_ESC_r - delta_ESC_y + delta_ESC_v
                #---------------------------------------------------------------------------
                # Apply the blended outputs to the ESC PWM signal -- thru GPIO
                #---------------------------------------------------------------------------
                #if esc_index == 4:
                esc.update(delta_spin)

            #---------------------------------------------------------------------------
            # Store telemetry data for post flight analysis
            #---------------------------------------------------------------------------
            if tel_save:
                dloop += 1
            if dloop == Dc:
                dloop = 0
                loop += 1
            if loop == Ns:
                loop = 0
            flt_dat[0][loop] = on_time #flight time (sec)
            flt_dat[1][loop] = statics_nav[0] #theta_x
            flt_dat[2][loop] = statics_nav[1] #theta_y
            flt_dat[3][loop] = statics_nav[2] #theta_z
            flt_dat[4][loop] = statics_nav[3] #vel_x
            flt_dat[5][loop] = statics_nav[4] #vel_y
            flt_dat[6][loop] = statics_nav[5] #vel_z
            flt_dat[7][loop] = statics_nav[6] #accel_x
            flt_dat[8][loop] = statics_nav[7] #accel_y
            flt_dat[9][loop] = statics_nav[8] #accel_z
            flt_dat[10][loop] = Gx #omega_x
            flt_dat[11][loop] = Gy #omega_y
            flt_dat[12][loop] = Gz #omega_z
            flt_dat[13][loop] = rx_alt #baro_alt delta from takeoff
            flt_dat[14][loop] = magx #magnetometer x
            flt_dat[15][loop] = magy #magnetometer y
            flt_dat[16][loop] = magz #magnetometer z

            #---------------------------------------------------------------------------
            # Writing of telemetry data to console
            #---------------------------------------------------------------------------
            if telem_flag:
                pindx += 1
            if pindx == 60: # writing out telemetry at ~5 Hz
                pindx = 0
            try:
                if telm_idx == 1:
                    print('%10d %10d %10d %10d %10d %10d %10d %10d' \
                    % (stk_r, stk_p, stk_v, stk_y, ch_5, ch_6, ch_7, ch_8)) # RC command inputs (1000-2000)
                #% (stk_r, stk_p, stk_v, stk_y, delta_ESC_v, delta_ESC_r, delta_ESC_p, delta_ESC_y))
                if telm_idx == 2:
                    tel_dat1 = rx_alt # baro alt
                    tel_dat2 = statics_nav[12] # filtered altitude (m)
                    tel_dat3 = alt_sp # altitude setpoint (m)
                elif telm_idx == 3:
                    tel_dat1 = statics_sensor[0] # measured heading (rad)
                    tel_dat2 = statics_nav[2] # filtered yaw angle (rad)
                    tel_dat3 = thet_yaw_sp # yaw angle setpoint (rad)
                elif telm_idx == 4:
                    tel_dat1 = Gx # raw gyro rate x (deg/sec)
                    tel_dat2 = Gy # raw gyro rate y (deg/sec)
                    tel_dat3 = Gz # raw gyro rate z (deg/sec)
                elif telm_idx == 5:
                    tel_dat1 = Ax # raw accelerometer x axis
                    tel_dat2 = Ay # raw accelerometer y axis
                    tel_dat3 = Az # raw accelerometer z axis
                    Nsum = 1
                    sumAx = 0
                    sumAy = 0
                    sumGz = 0
                if (telm_idx == 2 or telm_idx == 3 or telm_idx == 4 or telm_idx == 5):
                    print('%7.3f %7.3f %7.3f %7.3f %7.4f %8.4f %7.3f %7.4f %11.6f %11.6f %8.3f %2.0f %2.0f %7.4f %7.4f' \
                    % (tel_dat1, tel_dat2, tel_dat3,vthrust_sp,magx,statics_nav[5],statics_nav[11],statics_nav[19],lat,lon,gps_alt,cv, flt_state, on_time, umag))
                #% (tel_dat1, tel_dat2, tel_dat3, delta_ESC_v,delta_ESC_r,delta_ESC_p,delta_ESC_y,statics_nav[19],lat,lon,gps_alt,cv, flt_state, on_time, umag))
            except:
                pass
            # sum up gyro data for prop balancing
            Axcal = Ax - cal_data[0]
            Aycal = Ay - cal_data[1]
            Gzcal = Gz - cal_data[8]
            sumAx = sumAx + Axcal*Axcal
            sumAy = sumAy + Aycal*Aycal
            sumGz = sumGz + Gzcal*Gzcal
            Nsum += 1

            # if Shut-Down Mode selected, exit main loop
            if flt_state == 3:
                Ready = False

        #---------------------------------------------------------------------------
        # End of Main Loop
        #---------------------------------------------------------------------------

        #---------------------------------------------------------------------------
        # Flight shut down and storage of flight data
        #---------------------------------------------------------------------------
        current_folder_path, current_folder_name = os.path.split(os.getcwd())
        outfilepath_name = str(current_folder_path) + "/" + str(current_folder_name) + "/PiQuad/flt_data/data.txt"
        outFile = file(outfilepath_name, 'w')
        try:
            for loop in range(Ns):
                data_st = str(' ')
            for k in range(Nv):
                data_st += str(flt_dat[k][loop])
            if k<(Nv-1):
                data_st += ", "
                data_st = data_st + "\n"
                outFile.write(data_st)
        except:
            pass
    else:
        outFile.close()
        print('\n Successfully wrote flight telemetry data to file /PiQuad/flt_data/data.txt')

        for esc in esc_list:
            esc.update(0)
        print("\n Flight Ended \n")
        time.sleep(1.0)
        ShutdownRPIO()


