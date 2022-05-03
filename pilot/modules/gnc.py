
'''
*****************************************************************************
    Copyright (C) 2021 by Robotics In Flight, LLC.   (roboticsinflight.com)
                          David Haessig & Michael Vogas

    PythonPilot is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    PythonPilot is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see https://www.gnu.org/licenses.
    
    PythonPilot consists of the main routine -- PythonPilot_main_{yymmdd}.py -- 
    and subroutines contained in the following files:
      -- gnc.py     guidance, navigation and control functions
      -- flt_st.py  flight state machine functions
      -- esc.py     PWM control generation function
      -- imu.py     IMU read function
      -- kb_cmd.py  keyboard read function
 
    PythonPilot is a trademark of Robotics In Flight, LLC.
*****************************************************************************
'''

from __future__ import division
import sys
import os
import time
import datetime
import cython
import pyximport
import math
#from Crypto.Cipher import _Blowfish
from struct import pack
import modules.imu

def imu_avg(mpu6050):
    # average 10000 samples
    sax = 0
    say = 0
    saz = 0
    sgx = 0
    sgy = 0
    sgz = 0
    N = 1000 
    
    for cal_count in range(1,N) :
    
        # read the IMU
        C = mpu6050.ReadData()    # Rate and acceleration in deg/sec and G's respectively
        # Apply a coordinate conversion for the Prototype units (Proto 2&3) ****************
        # changing from x-front, y-left, z-up to x-front, y-right, z-down
        #Ax =  C.Gx
        #Ay = -C.Gy
        #Az = -C.Gz
        #Gx =  C.Gyrox
        #Gy = -C.Gyroy
        #Gz = -C.Gyroz
        # Apply a coordinate conversion for the Alpha-Beta units, **************************
        # changing from x-left, y-front, z-down to x-front, y-right, z-down
        Ax =  C.Gy
        Ay = -C.Gx
        Az =  C.Gz
        Gx =  C.Gyroy
        Gy = -C.Gyrox
        Gz =  C.Gyroz	
        # sum up raw output to compute averages
        sax += Ax
        say += Ay
        saz += Az
        sgx += Gx
        sgy += Gy
        sgz += Gz

    ax_mean = sax/N
    ay_mean = say/N
    az_mean = saz/N
    gx_mean = sgx/N
    gy_mean = sgy/N
    gz_mean = sgz/N
    print("[ax_mean ay_mean az_mean] = [" + str(ax_mean)+", "+str(ay_mean)+", "+str(az_mean)+"]  G's")
    print("[gx_mean gy_mean gz_mean] = [" + str(gx_mean)+", "+str(gy_mean)+", "+str(gz_mean)+"]  deg/sec")
    A = [ax_mean, ay_mean, az_mean]
    G = [gx_mean, gy_mean, gz_mean]

    return A, G
    
def cal_end(flag):
    print("\n********************************************************************************")
    if flag == 1:
        print(" IMU Calibration Aborted ")
    elif flag == 2:
        print(" IMU Calibration Completed - File written ")
    print("********************************************************************************")
    return	

def imu_cal(mpu6050,cal_data):

    # Inputs:
    # Ax, Ay, Az - IMU accelerations (G's)
    # Gx, Gy, Gz - IMU angular velocities (deg/sec)
    # Outputs:
    # file -- /PiQuad/calfiles/coef_file1.txt

    #-------------------------------------------------------------------------------------------
    # Static calibration of the IMU 
    #-------------------------------------------------------------------------------------------

    print("\n\n\n********************************************************************************")
    print(" Beginning IMU Calibration")
    print("********************************************************************************")
    print("  This will produce estimates of the IMU error coefficients and write these values")
    print("	 to: /PiQuad/calfiles/IMU_coef.txt, overwriting the existing file.")
    reply = raw_input("\n  Do you wish to proceed (y) or skip ([])?: ")
    if reply != 'y':
        cal_end(1)
        return
    
    print("****************************************************************")
    print("\n  Place the unit in upright position on level platform - z-axis down \n")
    raw_input("  Hit enter when ready\n")
    print("  Measuring IMU output when upright (z axis down)")
    [Azn,Gd] = imu_avg(mpu6050)

    print("****************************************************************")
    print("\n  Place the unit in an upside down position on level platform \n")
    raw_input("  Hit enter when ready\n")
    print("  Measuring IMU output when inverted (z axis up)")
    [Azp,Gu] = imu_avg(mpu6050)

    # Calibration equations - gyro biases
    bwx = (Gu[0] + Gd[0])/2
    bwy = (Gu[1] + Gd[1])/2
    bwz = (Gu[2] + Gd[2])/2 
    
    print("****************************************************************")
    print("\n  Place the unit such that the y-axis is up \n")
    raw_input("  Hit enter when ready\n")
    print("  Measuring IMU output when upright (z axis down)")
    [Ayp,Gd] = imu_avg(mpu6050)

    print("****************************************************************")
    print("\n  Place the unit such that the y-axis is down \n")
    raw_input("  Hit enter when ready\n")
    print("  Measuring IMU output when inverted (z axis up)")
    [Ayn,Gu] = imu_avg(mpu6050)

    print("****************************************************************")
    print("\n  Place the unit such that the x-axis is up \n")
    raw_input("  Hit enter when ready\n")
    print("  Measuring IMU output when upright (z axis down)")
    [Axp,Gd] = imu_avg(mpu6050)

    print("****************************************************************")
    print("\n  Place the unit such that the x-axis is down \n")
    raw_input("  Hit enter when ready\n")
    print("  Measuring IMU output when inverted (z axis up)")
    [Axn,Gu] = imu_avg(mpu6050)
    
    # Calibration coefficient equations - accelerometer parameters
    # Scalefactors
    SFx = (Axp[0] - Axn[0])/2
    SFy = (Ayp[1] - Ayn[1])/2
    SFz = (Azp[2] - Azn[2])/2
    # Biases
    bx1 = (Axp[0]+Axn[0])/2
    bx2 = (Ayp[0]+Ayn[0])/2
    bx3 = (Azp[0]+Azn[0])/2
    by1 = (Axp[1]+Axn[1])/2
    by2 = (Ayp[1]+Ayn[1])/2
    by3 = (Azp[1]+Azn[1])/2
    bz1 = (Axp[2]+Axn[2])/2
    bz2 = (Ayp[2]+Ayn[2])/2
    bz3 = (Azp[2]+Azn[2])/2
    bax = (bx1+bx2+bx3)/3
    bay = (by1+by2+by3)/3
    baz = (bz1+bz2+bz3)/3
    # Angle tilts
    psi1 = -(Axp[1] - Axn[1])/(2*SFy)
    psi2 =  (Ayp[0] - Ayn[0])/(2*SFx)
    alpha_z = (psi1 + psi2)/2
    the1 =  (Axp[2] - Axn[2])/(2*SFz)
    the2 = -(Azp[0] - Azn[0])/(2*SFx)
    alpha_y = (the1 + the2)/2
    phi1 = -(Ayp[2] - Ayn[2])/(2*SFz)
    phi2 =  (Azp[1] - Azn[1])/(2*SFy)
    alpha_x = (phi1 + phi2)/2
    
    print("\n****************************************************************")
    print("IMU Calibration Results:")
    print("   Rate bias         = " + str(bwx) + "  " + str(bwy) + "  " + str(bwz) + "   deg/sec")
    print("   Accel bias        = " + str(bax) + "  " + str(bay) + "  " + str(baz) + " G's")
    print("   Tilt angle        = " + str(alpha_x) + "  " + str(alpha_y) + "  " + str(alpha_z) + " rad")
    print("   Accel Scalefactor = " + str(SFx) + "  " + str(SFy) + "  " + str(SFz))
    print("****************************************************************\n")

    reply = raw_input("  Do you wish to overwrite the existing calibration file? (y / n):")
    if reply != 'y':
        cal_end(1)
        return cal_data
        
    # Write out the calibration coefficient files
    # IMU Calibration coefficients
    try:
        current_folder_path, current_folder_name = os.path.split(os.getcwd())
        outfilepath_name = str(current_folder_path) + "/" + str(current_folder_name) + "/PiQuad/calfiles/IMU_coef.txt"
        #print outfilepath_name
        outFile = file(outfilepath_name, 'w')
        datetime_now = datetime.datetime.strftime(datetime.datetime.now(), '%Y-%m-%d %H:%M:%S')
        cal_data = [bax, bay, baz, SFx, SFy, SFz, bwx, bwy, bwz, 1, 1, 1, alpha_x, alpha_y, alpha_z]
        cal_data_st = datetime_now + "," + str(bax)  + "," + str(bay) + "," + str(baz) + "," + str(SFx) + "," + str(SFy) + "," +  str(SFz) + \
            "," +  str(bwx) + "," +  str(bwy) + "," +  str(bwz) + ",1,1,1," + str(alpha_x) + "," +  str(alpha_y) + "," +  str(alpha_z)
        #print "cal_data_st:  " + cal_data_st
        outFile.write(cal_data_st)
        outFile.close()
        print('\n Successfully wrote contents of IMU calibration file created on ' + datetime_now)
        #print '     IMU cal data --> ' + str(cal_data)
    except:
        print('\n Writing of the IMU calibration file was not successfully')
        print('     IMU cal data set to nominal values')
        cal_data = [0,0,0,1,1,1,0,0,0,1,1,1,0,0]
        print('     IMU cal data --> ' + str(cal_data))

    cal_end(2)
    
    return cal_data


def gnc_init():

    # IMU Calibration coefficients
    try:
        current_folder_path, current_folder_name = os.path.split(os.getcwd())
        infilepath_name = str(current_folder_path) + "/" + str(current_folder_name) + "/PiQuad/calfiles/IMU_coef.txt"
        inFile = file(infilepath_name, 'r')
        contents = inFile.read()
        inFile.close()
        str_data = contents.split(',')
        datetime = str_data[0]
        cal_data = []
        for i in range(1, 15):
            cal_data.append(float(str_data[i]))
        print("\n********************************************************************************")
        print(' Successfully read the IMU calibration file')
        print('    IMU cal data --> Created on ' + datetime)
        print('       Accel Bias(G) & SF:       %7.4f %7.4f %7.4f %7.4f %7.4f %7.4f' % tuple(cal_data[0:6]))
        print('       Gyro Bias (deg/s) & SF:   %7.4f %7.4f %7.4f %7.4f %7.4f %7.4f' % tuple(cal_data[6:12]))
        print('       IMU Frame Offsets (rad):  %7.4f %7.4f' % tuple(cal_data[12:14]))
        print("********************************************************************************\n")
    except:
        print("\n********************************************************************************")
        print(' Reading of the IMU calibration file was not successfully')
        print('     IMU cal data set to nominal values')
        cal_data = [0,0,0,1,1,1,0,0,0,1,1,1,0,0]
        print("     IMU cal data --> ,%2d %2d %2d %2d %2d %2d %2d %2d %2d %2d %2d %2d %2d %2d" % (0,0,0,1,1,1,0,0,0,1,1,1,0,0) )
        print("********************************************************************************")

# Heading Calibration coefficients
    try:
        current_folder_path, current_folder_name = os.path.split(os.getcwd())
        infilepath_name = str(current_folder_path) + "/" + str(current_folder_name) + "/PiQuad/calfiles/mag_cal.txt"
        inFile = file(infilepath_name, 'r')
        contents = inFile.read()
        inFile.close()
        str_data = contents.split(',')
        datetime = str_data[0]
        mag_cal = []
        for i in range(1,5):
            mag_cal.append(float(str_data[i]))
        print("********************************************************************************")
        print(" Successfully read the magnetometer calibration file")
        print("   Magnetometer bias readings: " + str(mag_cal[0:3]) + " counts")
        print("   Magnetometer heading angle reading at true north: " + str(mag_cal[3]*57.3) + " degrees")
        print("********************************************************************************\n")
    except:
        print("\n********************************************************************************")
        print(' Reading of the magnetometer calibration file was not successfully')
        print(" Operation aborted due to magnetometer calibration file read failure")
        print("********************************************************************************")
        # temporary debug
        print(infilepath_name)
        exit()

    return cal_data, mag_cal

def nav(Ax, Ay, Az, Gx, Gy, Gz, mpu6050, delta_time, st_list, cal_data, rx_alt, magx, magy, magz, mag_cal, statics_sensor, cv):

    # Inputs:
        # Ax, Ay, Az - IMU accelerations (G's)
        # Gx, Gy, Gz - IMU angular velocities (deg/sec)
        # Outputs:
        # xr_k, xp_k, xy_k - Estimated roll, pitch, and yaw angles (rad)
        # vx_k, vy_k, vz_k - Estimated linear velocities along x, y, and z axes (m/sec)
        # bax_k, bay_k, baz_k - Estimated accelerometer biases along x, y, and z axes (m/sec^2)
        # sfax_k, sfay_k, sfaz_k - Estimated accelerometer scalefactor along x, y, and z axes (-)
        # bwx_k, bwy_k, bwz_k - Estimated rate biases along x, y, and z axes (rad/sec)
        # pz_k - z (altitude) position estimate

    #===================================================================================
    # Static variables
    #===================================================================================
    xr_k = st_list[0]
    xp_k = st_list[1]
    xy_k = st_list[2]
    vx_k = st_list[3]
    vy_k = st_list[4]
    vz_k = st_list[5]
    ax_k = st_list[6]
    ay_k = st_list[7]
    az_k = st_list[8]
    bax_k = st_list[9]
    bay_k = st_list[10]
    baz_k = st_list[11]
    pz_k  = st_list[12]
    bwx_k = st_list[13]
    bwy_k = st_list[14]
    bwz_k = st_list[15]
    sfax_k = st_list[16]
    sfay_k = st_list[17]
    sfaz_k = st_list[18]

    #===================================================================================
    # IMU & Nav Filter Coefficients
    #===================================================================================
    # Coarse calibration coefficients 
    # Acceleration biases in G's
    b_ax = cal_data[0]
    b_ay = cal_data[1]
    b_az = cal_data[2]
    sf_ax = cal_data[3]
    sf_ay = cal_data[4]
    sf_az = cal_data[5]
    # Gyro biases in deg/sec
    b_gx = cal_data[6]
    b_gy = cal_data[7]
    b_gz = cal_data[8]
    sf_gx = cal_data[9]
    sf_gy = cal_data[10]
    sf_gz = cal_data[11]
    # IMU tilt coefficients
    alpha_x = cal_data[12]
    alpha_y = cal_data[13]
    # Gravity (m/s^2)
    g = 9.8067		
    # Angular rate conversion coefficient -- 500 deg/sec per 2^15 counts -- converted to (rad/sec / count)
    Kw = (3.14159/180)    #(500/32768)*(3.14159/180)
    # Linear acceleration conversion -- ((m/s^2) / g )
    Ka = g
    
    # altitude estimation coefficients
    K1 = 0.043  #0.0093
    K2 = 0.29   #0.1363
    K3 = 0.0032 #0.0003
    if cv == 3:
        K1 = 0.093*10
        K2 = 1.363*10
        K3 = 0.003*10
    
    # horizontal velocity estimation coefficients
    Kh1 = 0.0251
    Kh2 = 0.0003
    
    # attitude filter coefficients
    Katt1 = 0.1
    Katt2 = -0.01
    if cv == 3:
        Katt1 = 1.0
        Katt2 = -0.1

    # Magnetic flux sensor output calibration coefficients   (to be read in from file)
    # Bias offsets  (fixed temp values for PiQuad #2 - Daves) ??????????????????????   FIX FIX  FIX   FIX ??????????????????????????   FIX  FIX   FIX
    #b_magx = 0
    #b_magy = +140
    #b_magz = 0

    #===================================================================================
    # IMU Coarse Compensation
    #===================================================================================
    # Accelerometer coarse compensation equations -- correcting for bias and scale factor
    Ax = (Ax - b_ax)/sf_ax
    Ay = (Ay - b_ay)/sf_ay
    Az = (Az - b_az)/sf_az

    # Gyro compensation equations -- correcting for bias and scale factor
    Gx = (Gx - b_gx)/sf_gx
    Gy = (Gy - b_gy)/sf_gy
    Gz = (Gz - b_gz)/sf_gz
        
    # Accelerometer and gyro axes tilt compensation
    Abx = Ax + alpha_y*Az
    Aby = Ay - alpha_x*Az
    Abz = Az - alpha_y*Ax + alpha_x*Ay
    Gbx = Gx + alpha_y*Gz
    Gby = Gy - alpha_x*Gz
    Gbz = Gz - alpha_y*Gx + alpha_x*Gy
                
    # Convert body angular rates from deg/sec to radians/sec
    omega_r = Kw * Gbx
    omega_p = Kw * Gby
    omega_y = Kw * Gbz
    
    # Convert body accelerations from G's to m/sec^2 
    Abx = Ka * Abx
    Aby = Ka * Aby
    Abz = Ka * Abz
    
    # Average last 2 accelerometer readings
    Abxf = (Abx + ax_k)/2
    Abyf = (Aby + ay_k)/2
    Abzf = (Abz + az_k)/2
    
    #===================================================================================
    # Magnetic Sensor Coarse Compensation
    #===================================================================================
    b_magx = mag_cal[0]
    b_magy = mag_cal[1]
    b_magz = mag_cal[2]
    heading_bias = mag_cal[3]
    
    # magnetic sensor bias compensation
    magx = magx - b_magx
    magy = magy - b_magy
    magz = magz - b_magz
    
    # coordinate transformation to remove heading bias error
    ctb = math.cos(heading_bias)
    stb = math.sin(heading_bias)
    mxp = ctb*magx - stb*magy
    myp = stb*magx + ctb*magy
    magx = mxp
    magy = myp
    #print "   " + str(magx) + ",    " + str(magy) + ",   " + str(magz)
    #===================================================================================
    # Inertial Navigation
    #===================================================================================
    # convert the accelerometer readings to angles under assumption of zero horizontal acceleration
    theta_pitch, theta_roll  = mpu6050.getTiltAngles(Abxf, Abyf, Abzf)

    # Euler angle dynamic equations (roll pitch yaw)
    cphi = math.cos(xr_k)
    sphi = math.sin(xr_k)
    cthe = math.cos(xp_k)
    sthe = math.sin(xp_k)
    cpsi = math.cos(xy_k)
    spsi = math.sin(xy_k)
    #phi_dot = omega_r + (omega_p*sphi + omega_y*cphi)*sthe/cthe
    the_dot = (omega_p*cphi - omega_y*sphi)
    psi_dot = (omega_p*sphi + omega_y*cphi)/cthe
    phi_dot = omega_r + psi_dot*sthe

    # Roll angle and rate bias extended kalman filter
    dT = delta_time
    r = theta_roll - xr_k
    xr_kp1 = xr_k +  dT*((phi_dot - bwx_k) + Katt1*r)
    xr_k = xr_kp1
    bwx_k = bwx_k + dT*Katt2*r
    
    # Pitch angle and rate bias extended kalman filter
    r = theta_pitch - xp_k
    xp_kp1 = xp_k +  dT*((the_dot - bwy_k) + Katt1*r)
    xp_k = xp_kp1
    bwy_k = bwy_k + dT*Katt2*r

    # Convert fine compensated body acceleration (Abx Aby Abz)' to inertial axes
    #cphi = math.cos(xr_kp1)
    #sphi = math.sin(xr_kp1)
    #cthe = math.cos(xp_kp1)
    #sthe = math.sin(xp_kp1)
    #cpsi = math.cos(xy_kp1)
    #spsi = math.sin(xy_kp1)
    c11 = cpsi*cthe
    c12 = cpsi*sthe*sphi - spsi*cphi
    c13 = cpsi*sthe*cphi + spsi*sphi
    c21 = spsi*cthe
    c22 = spsi*sthe*sphi + cphi*cpsi       #+ cthe*cphi   (fix 3/11/17 dah)
    c23 = spsi*sthe*cphi - cpsi*sphi
    c31 = -sthe
    c32 = cthe*sphi
    c33 = cthe*cphi
    Aix = c11*Abxf + c12*Abyf + c13*Abzf
    Aiy = c21*Abxf + c22*Abyf + c23*Abzf
    Aiz = c31*Abxf + c32*Abyf + c33*Abzf
    
    # Vertical altitude and velocity EKF with z-axis positive UP (i.e. positive rx_alt)
    # Compute altitude residual
    try:
        r = rx_alt - pz_k
    except:
        r = 0
    res_alt = r
    vz_kp1 = vz_k + ( -(Aiz + g) + K1*r + baz_k)*dT
    pz_kp1 = pz_k + (vz_k + K2*r)*dT
    baz_kp1 = baz_k + K3*r*dT
    #print "Alt filt: res = " + str(r) + " vel_hat = " + str(vz_kp1) + " x_hat = " + str(pz_kp1) + " bias = "  + str(baz_kp1) + " accel = " + str(-(Aiz))+ " accel error = " + str(-(Aiz+g) + baz_k)
        
    # Horizontal velocity filters
    vx_kp1 = vx_k + ((Aix + ax_k)/2 - Kh1*vx_k)*dT
    #bax_kp1 = bax_k - K3*vx_k*dT
    vy_kp1 = vy_k + ((Aiy + ay_k)/2 - Kh1*vy_k)*dT
    #bay_kp1 = bay_k - K3*vy_k*dT
    
    
    # Yaw sensor processing and filtering ***********************************
    rx_head_good = 1
    # transform magnetometer readings onto local level frame
    try:
        c11 = cthe
        c12 = sthe*sphi
        c13 = sthe*cphi
        c21 = 0
        c22 = cthe 				# cthe*cphi    (fix 3/11/17 dah)
        c23 = -sphi				# sphi
        c31 = -sthe
        c32 = cthe*sphi
        c33 = cthe*cphi
        magix = c11*magx + c12*magy + c13*magz
        magiy = c21*magx + c22*magy + c23*magz
        magiz = c31*magx + c32*magy + c33*magz
        heading_true = -math.atan2(magiy, magix) # heading in radians
        statics_sensor[0] = heading_true 
    except:	
        #print "heading calc issue"
        rx_head_good = 0 
        
    # Yaw angle and yaw rate bias Extended Kalman Filter
    #   - produces filtered heading angle provided by CRIUS compass
    if rx_head_good == 1:
        try:
            r = heading_true - xy_k
            while r > math.pi:
                r -= 2*math.pi
            while r < -math.pi:
                r += 2*math.pi
        except:
            r = 0
    else:
        r = 0
    xy_kp1 = xy_k +  dT*((psi_dot - bwz_k) + Katt1 * r )
    xy_k = xy_kp1
    bwz_k = bwz_k + dT*Katt2*r
    
    #if rx_head_good == 1:
    #    print " heading_true = " + str(heading_true) + " magx = " + str(magx) + " magix = " + str(magix) + " magy = " + str(magy) + " magiy = " + str(magiy)

    # retain the previous velocity and acceleration values for the next pass 
    vx_k = vx_kp1
    vy_k = vy_kp1
    vz_k = vz_kp1
    ax_k = Abx
    ay_k = Aby
    az_k = Abz
    # retain the previous z position and acceleration bias & scalefactor for the next pass
#    if cv== 2:
#        bax_k = bax_kp1
#        bay_k = bay_kp1
    baz_k = baz_kp1
#        sfax_k = sfax_kp1
#        sfay_k = sfay_kp1
#        sfaz_k = sfaz_kp1
    pz_k = pz_kp1 

    delta_g = -(Aiz + g)
    statics_nav = [xr_k, xp_k, xy_k, vx_k, vy_k, vz_k, ax_k, ay_k, az_k, bax_k, bay_k, baz_k, pz_k, bwx_k, bwy_k, bwz_k, sfax_k, sfay_k, sfaz_k, delta_g, theta_pitch, theta_roll, Aiz, res_alt]
        
    return omega_r, omega_p, omega_y, vx_kp1, vy_kp1, vz_kp1, statics_nav, statics_sensor
    

def control(cv, thet_pitch_sp, thet_roll_sp, omeg_yaw_sp, thet_yaw_sp, alt_sp, xr_kp1, xp_kp1, xy_kp1, omega_r, omega_p, omega_y, vx_kp1, vy_kp1, vz_kp1, pz_k, statics_cnt):

    x1p_k = statics_cnt[0]
    x2p_k = statics_cnt[1]
    x3p_k = statics_cnt[2]
    x1r_k = statics_cnt[5]
    x2r_k = statics_cnt[6]
    x3r_k = statics_cnt[7]
    
    # LQR / LQG compensator coefficients  (3.3 Hz bandwidth)
    ad11 = 0.951
    ad12 = 9.651e-5
    ad13 = 2.301e-5
    ad21 = -0.03146
    ad22 = 0.54
    ad23 = 0.3142
    ad31 = -0.1384
    ad32 = -0.2243 
    ad33 = 0.789
    bd11 = 0.049
    bd12 = 0.004780
    bd21 = 0.003161
    bd22 = 0.4547
    bd31 = -0.003926
    bd32 = 0.1978
    cd1 = 3.162
    cd2 = 0.58
    cd3 = 2.398
        
    # Yaw rate controller gain
    Kd = 1.8
    
    # Yaw angle controller gains
    Kp_yaw=1.8      # proportional gain (N-m / rad)
    Kd_yaw=1.8      # derivative gain (N-m / (rad/sec) )

    #===================================================================================
    # Controller
    #===================================================================================		

    # Pitch LQR/LQG controller *********************************************************
    y1pk = xp_kp1 - thet_pitch_sp 
    y2pk = omega_p
    x1p_kp1 = ad11*x1p_k + ad12*x2p_k + ad13*x3p_k + bd11*y1pk + bd12*y2pk
    x2p_kp1 = ad21*x1p_k + ad22*x2p_k + ad23*x3p_k + bd21*y1pk + bd22*y2pk
    x3p_kp1 = ad31*x1p_k + ad32*x2p_k + ad33*x3p_k + bd31*y1pk + bd32*y2pk
    u_pitch = -(cd1*x1p_kp1 + cd2*x2p_kp1 + cd3*x3p_kp1)
    x1p_k = x1p_kp1
    x2p_k = x2p_kp1
    x3p_k = x3p_kp1

    # Roll LQR/LQG controller *********************************************************
    y1rk = xr_kp1 - thet_roll_sp
    y2rk = omega_r
    x1r_kp1 = ad11*x1r_k + ad12*x2r_k + ad13*x3r_k + bd11*y1rk + bd12*y2rk
    x2r_kp1 = ad21*x1r_k + ad22*x2r_k + ad23*x3r_k + bd21*y1rk + bd22*y2rk
    x3r_kp1 = ad31*x1r_k + ad32*x2r_k + ad33*x3r_k + bd31*y1rk + bd32*y2rk
    u_roll = -(cd1*x1r_kp1 + cd2*x2r_kp1 + cd3*x3r_kp1)
    x1r_k = x1r_kp1
    x2r_k = x2r_kp1
    x3r_k = x3r_kp1
    
    # Yaw rate controller *********************************************************
    if cv == 0:
        u_yaw = Kd*(omeg_yaw_sp - omega_y)
    # Yaw angle controller *********************************************************
    else:
        u_yaw = Kp_yaw*(thet_yaw_sp-xy_kp1) - Kd_yaw*omega_y

    # Altitude Controller *********************************************************
    G1 = 123
    G2 = 100
    u = (-G2*vz_kp1 + G1*(alt_sp - pz_k))
    
    if u < 0:
        u = 0
    elif u > 700:
        u = 700
    u_vert = u

    statics_cntr = [x1p_k, x2p_k, x3p_k, 0, 0, x1r_k, x2r_k, x3r_k, 0, 0]

    return u_pitch, u_yaw, u_roll, u_vert, statics_cntr


# ---------------------------------------------------------------------
# RC Stick Input Processing Routines
# ---------------------------------------------------------------------
def rc_trk(rx_in, xk, D):
    # RC Command Input Tracking Adaptive Deadband function
    # Reduces command input noise by applying a deadband 
    # about current temporal command input setting
    # inputs:
    #     rx_in - raw input from radio controller
    #     xk - static variables list
    # outputs:
    #     x0 - filtered output variables
    #     xk - updated static variable

    xk[3] = xk[2]
    xk[2] = xk[1]
    xk[1] = xk[0]
    xk[0] = rx_in
    yk = (xk[0]+xk[1]+xk[2]+xk[3])/4

    xo = yk
    if ((rx_in - yk) > D):
        xo = rx_in - D
    elif ((rx_in - yk) < -D):
        xo = rx_in + D

    return xo, xk

    
def sign(x): return 1 if x >= 0 else -1

def rc_trk1(rx_in, xk, D):
    # RC Command Input Tracking Adaptive Deadband function #2
    # Reduces command input noise by applying a deadband 
    # about current temporal command input setting
    # inputs:
    #     rx_in - raw input from radio controller
    #     xk - static variables list
    # outputs:
    #     x0 - filtered output variables
    #     xk - updated static variable
    
    #Dlt = rx_in - xk[0]
    #if abs(Dlt) < D:
    uk = (xk[3]+xk[2]+rx_in)/3
    xk[3] = xk[2]
    xk[2] = rx_in
    y1k = 0.92*xk[1] + 0.08*uk
    xk[1] = y1k
    y2k = 0.92*xk[0] + 0.08*y1k
    xk[0] = y2k
    xo = y2k

    Dlt = rx_in - xk[0]
    if abs(Dlt) > D:
        xo = rx_in - D*sign(Dlt)
        xk = [rx_in, rx_in, rx_in, rx_in]
    else:
        xo = y2k
    #print "Inside = "+ str(rx_in) + " out = " + str(xo)
    
    return xo, xk
    
    
def rc_deadband(rx_in, D):
    # Standard deadband function applied around zero
    # input:
    #     rx_in - raw input from radio controller
    # output:
    #     x0 - filtered output variables
    #D = 40		# deadband range in RC counts
    xo = 0
    if (rx_in > D):
        xo = rx_in - D
    elif (rx_in < -D):
        xo = rx_in + D
    return xo	
    
    
# ---------------------------------------------------------------------
# Setpoint
# ---------------------------------------------------------------------
def setpoint(cv, stk_r, stk_p, stk_y, stk_v, ch_5, ch_6, ch_7, ch_8, statics_sp, rc_cal_data):
    # This function receives raw stick data and computes the setpoint values and switch integer values
    # The range of stk outputs values are approximately:
        # -1 to +1 for roll, pitch, & yaw
        #  0 to +2 for vertical
    # The range for the 'Aux2' knob is approximately:
        # 0 to 1 and is currently unused
    # The switch values:
        # sw5 = {0, 1}
        # sw6 = {0, 1, 2}
        # sw8 = {0, 1}

    # Roll axis
    D = 40   # deadband range (+-D) in RC counts
    xk = statics_sp[0:4]
    #[rx_0, xk] = rc_trk(stk_r, xk, D)
    [rx_0, xk] = rc_trk1(stk_r, xk, D)
    sp_r = (rx_0 - rc_cal_data[0][0]) / rc_cal_data[1][0] *2
    statics_sp[0:4] = xk

    # Pitch axis
    xk = statics_sp[4:8]
    #[rx_0, xk] = rc_trk(stk_p, xk, D)
    [rx_0, xk] = rc_trk1(stk_p, xk, D)
    sp_p = (rx_0 - rc_cal_data[0][1]) / rc_cal_data[1][1] *2
    #print "Test: " + str(stk_p) + " " + str(rx_0) + " " + str(sp_p)
    statics_sp[4:8] = xk

    if cv == 0:
        # Yaw axis -- adaptive dead band
        xk = statics_sp[8:12]
        [rx_0, xk] = rc_trk(stk_y, xk, D)
        sp_y = (rx_0 - rc_cal_data[0][2]) / rc_cal_data[1][2] *2
        statics_sp[8:12] = xk
        
        # Vertical axis adaptive deadband 
        xk = statics_sp[12:16]
        [rx_0, xk] = rc_trk(stk_v, xk, D)
        #print "  " + str(stk_v) + "   " + str(rx_0)
        sp_v = (rx_0 - rc_cal_data[0][3]) / rc_cal_data[1][3] *2
        statics_sp[12:16] = xk
        
    else:
        # Yaw axis -- fixed dead band
        D = 30
        sp_y = rc_deadband((stk_y - rc_cal_data[0][2]), D) / (rc_cal_data[1][2]-2*D) *2
        #print "sp_y: " + str(stk_y) + "  " + str(rc_cal_data[0][2]) + "  " + str(rc_cal_data[1][2])
        # Vertical axis -- scale setpoint from 0 to 2 
        sp_v = (stk_v - rc_cal_data[0][3]) / (rc_cal_data[1][3]-2*D) *2        
        # Rotary knob input channel 7
        #print "ch_7: " + str(ch_7) + " " + str(1000)
        sp7 = (float(ch_7) - 1000) / 1000
        #print "SP7: " + str(sp7)
        
    # Switches 5, 6, and 8
    # switch inputs from Turnigy 9x run typically from 1000 to 1900 when switched
    sw5 = 0
    sw6 = 0
    sw8 = 0
    if ch_5>1450:
        sw5 = 1
    if ch_6>1250:
        sw6 = 1
    if ch_6>1650:
        sw6 = 2
    if ch_8>1450:
        sw8 = 1

    return sp_r, sp_p, sp_y, sp_v, sw5, sw6, sp7, sw8, statics_sp

    
# ---------------------------------------------------------------------
# Guidance
# ---------------------------------------------------------------------
def guidance(cv, sp_r, sp_p, sp_y, sp_v, statics_guid, theta_yaw, flt_state):
    # static_guid:
    #    static_guid[0] - heading angle setpoint (rad)
    #    static_guid[1] - altitude setpoint (m)

    thet_roll_sp = sp_r * 0.44 						# (rad)  with range of +/- 25 degrees
    thet_pitch_sp = -sp_p * 0.44					# (rad)  with range of +/- 25 degrees
    d_omeg_yaw_sp = 0 
    MAX_ALT = 110									# (meters)  110 meters = 360 ft
    
    if cv == 0:
        omeg_yaw_sp = sp_y / 1						# (rad/sec)  with range of +/- 30 degrees/sec
        vthrust_sp = sp_v * 500						# (ESC counts) with range of 0 to 1000 counts (msec)
        alt_sp = 0
        thet_yaw_sp = 0
    else:											# enter here is control variable cv == 1 
        vthrust_sp = 0
        #thet_yaw_sp = 0 
        omeg_yaw_sp = 0

        # Heading angle setpoint   (rad) **************************************************************************
        if flt_state < 1:  # initialize setpoint to measured heading angle while in Pre-flight Cal and Idle Mode
            thet_yaw_sp = theta_yaw
            statics_guid[0] = thet_yaw_sp
        elif flt_state == 1:  # integrate yaw setpoint to produce heading setpoint command
            dsp_y = rc_deadband(sp_y, 0.2)
            thet_yaw_sp = statics_guid[0] + 0.13*dsp_y		# integrator (0.13*0.8*5samp/sec = 0.6 rad/sec = 30 deg/sec max angular velocity)
            statics_guid[0] = thet_yaw_sp
            #print "guid Yaw: " + str(thet_yaw_sp) + "  " + 	 str(dsp_y)
        elif flt_state > 1:    # if in Auto-Land or other modes, hold heading setpoint constant
            thet_yaw_sp = statics_guid[0]


        # Altitude setpoint computation  (meters)  ****************************************************************
        if flt_state < 1:
            alt_sp = statics_guid[1]
        elif flt_state == 1:
            dsp_v = rc_deadband((sp_v - 1.0), 0.2)
            if dsp_v < -0.5:
                dsp_v = -0.5;
            alt_sp = statics_guid[1] + 0.4*dsp_v		    # integrator (0.4*0.8*5samp/sec = 1.6 m/s max vert upward velocity)
                                                            #            (0.4*(-0.5)*5      = -1.0 m/s max downward velocity)
        elif flt_state == 2:   # in Auto-Land Mode, drop altitude setpoint by 1 m/s until the last 3 meters where drop to 0.5 m/s^2
            if statics_guid[1] > 4:
                dsp_v = -0.5		    # integrator (0.4*0.5*5samp/sec = 1.0 m/s downward velocity)
            else:
                dsp_v = -0.1            # integrator (0.4*0.1*5samp/sec = 0.15 m/s downward velocity)
            alt_sp = statics_guid[1] + 0.4*dsp_v
        elif flt_state > 2:
            alt_sp = statics_guid[1]
        if alt_sp > MAX_ALT:
            alt_sp = MAX_ALT
        elif alt_sp < -5:
            alt_sp = -5 
        statics_guid[1] = alt_sp
        #print "guid Vert: " + str(alt_sp) + "  " + 	 str(sp_v) + "  " + 	 str(dsp_v)

    return thet_roll_sp, thet_pitch_sp, thet_yaw_sp, omeg_yaw_sp, vthrust_sp, alt_sp, statics_guid
