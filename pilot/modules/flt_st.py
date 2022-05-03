
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

import math
from pilot.modules.gnc import imu_cal
from rif_cmd import heading_cal
from rif_cmd import rc_cal

# ---------------------------------------------------------------------
# Flight State Machine 
# ---------------------------------------------------------------------
def banner():
    print(" Enter one of the following PythonPilot Flight State Change Commands:")
    print("     Hit 1 to enter Flight Mode")
    print("     Hit 0 to return to Idle Mode")
    print("     Hit 'c' to enter Calibration Mode")
    print("     In Flight Mode after landing hit 0 to enter Idle Mode")
    print("     In Idle Mode hit 1 to return to Flight Mode")
    print("********************************************************************************\n")
	
def statemachine(flt_state, on_time, char, sp_r, sp_p, sp_y, sp_v, sw5, sw6, sw8, umag, statics_rc, \
        mpu6050, rc_port, serial_port_index, cal_data, RC_cal_data, mag_cal, telem_flag, telm_idx, cv):
	# Inputs:
		# flt_state - current flight state
		# char - character from keyboard via WiFi and PuTTy (console)
		# sp_x - roll, pitch, yaw, and vertical setpoints
        # swx - switch inputs
		# umag - control magnitude
		# statics_rc - radio control input command static variables
        # mpu6050 - handle to the IMU
        # rc_port, serial_port_index -- data for reading the RC serial link
        # cal_data - IMU cal data
        # RC_cal_data - RC cal data
        # mag_cal - magnetometer cal data
        # telem_flag - telemetry output to console ON/OFF flag
        # telm_idx - index controlling the output of telemetry data message content
        # cv - control variable

	# Outputs:
		# flt_state - new flight state
		# statis_rc - updated statics_rc
		# telem_flag - plotted telemetry on/off flag
		# telem_index - plotted telemetry control index
		# cv - control type index
        # RC_cal_data - RC calibration data
        # em_off - emergency (motors temporarily) off flag
        # shut_down - normal shut down flag 

    #print "sp: "+str(sp_r)+" "+str(sp_p)+" "+str(sp_y)+" "+str(sp_v)+" "+str(sw5)+" "+str(sw6)+" "+str(sw8)
    
    # ---------------------------------------------------------------------
    # Check for Flight State Change commands entered through Radio Controller
    # Commented out - replaced by mode changes intered through Switch 5
    # ---------------------------------------------------------------------
	## If in IDLE mode, test for RC Command == Flight Mode
    #if flt_state == 0:
    #    if (sp_v < 0.1 and sp_y < -0.7 and abs(sp_r)<0.1 and abs(sp_p)<0.1):
    #        statics_rc[0] += 1
    #        #print "statics_rc0 Up: " + str(statics_rc[0])
    #    else:
    #        statics_rc[0] = 0
    #        statics_rc[1] = 0
    #        #print "statics_rc0 Dn: " + str(statics_rc[0])
    #    if statics_rc[0] > 11:
    #        statics_rc[1] = 1
    #        print "Switching to Flt Mode via RC"

	## If in FLIGHT Mode, test for RC Command == Idle Mode
    #if flt_state == 1:
    #    if (sp_v < 0.1 and abs(sp_y) < 0.1 and sp_r > 0.7 and sp_p > 0.7):
    #        statics_rc[0] += 1
    #        #print "statics_rc0 Idle Dn: " + str(statics_rc[0]) + "Idle " + str(idle_off)
    #    else:
    #        statics_rc[0] = 0
    #    if statics_rc[0] > 11:
    #        statics_rc[1] = 0
    #        print "Switching to Idle Mode via RC"

    # ---------------------------------------------------------------------
    # Update Flight State variable(flt_state) based on RC stick input commands,
    # on-time, and RC switch inputs (SW6 and SW8)
    # keyboard command inputs removed
    # ---------------------------------------------------------------------
    
	# If not in Shut-Down Mode, test for RC Stick Command == Shut-Down Mode
    if flt_state != 3:
        if (sp_v < 0.1 and sp_y > 0.9):
            #print "sp: " + str(sp_v) + " " + str(sp_y) + " " + str(sp_p) + " " + str(sp_r)
            if sp_p < -0.7 and sp_r < -0.7:
                statics_rc[2] += 1
            else:
                statics_rc[2] = 0
            if statics_rc[2] > 5:
                statics_rc[1] = 3
                print("Switching to Shut-Down Mode via RC")   
                
    em_off = False
	# From Preflight Cal Mode go to Idle Mode after 45 seconds
    if flt_state == -1:
        #print "on-time = " + str(on_time)
        #cv = 3      # enable fast pre-flight calibration
        if on_time > 45:
            # Idle Mode
            flt_state = 0
            #cv = 1     #switch to normal flight mode
            print("Preflight Calibration Complete; switching to Idle Mode")
            banner()
            
	# From any state except 'Cal' switch to Shut-Down (Permanent OFF) mode when commanded 
    if statics_rc[1] == 3:
        flt_state = 3
        print("Switched to Shut-Down Mode")

	# From Idle go either to Cal Mode or Flight Mode
    elif flt_state == 0:
        if char == 'c':
            # Calibration Mode
            #flt_state = 4 
            cal_data = imu_cal(mpu6050,cal_data)
            mag_cal = heading_cal(rc_port, serial_port_index, mag_cal)
            RC_cal_data = rc_cal(rc_port, serial_port_index, RC_cal_data)
            #flt_state = 0
            banner()
        #elif char == '1' or statics_rc[1] == 1 or sw6 == 1:
        elif sw6 == 1:
            # Flight Mode
            if umag < 100:    # control command level must be less than 100 counts
                flt_state = 1
                statics_rc[1] = -1
                print("Switched to Flt Mode")
            else:
                #pass
                print("Switch to Flight Mode blocked due to unsafe prop command level")
                #print "umag = " + str(umag)
    # From Flight Mode go to Idle, Auto-Land or Emergency Off Mode
    elif flt_state == 1:
        #if char == '0' or statics_rc[1] == 0 or sw6 == 0:
        if sw6 == 0:        
            if umag < 350:    # control command level must be less than 350 counts to indicate landed condition
                flt_state = 0
                statics_rc[1] = -1
                print("Switched to Idle Mode")
            else:
                print("Switch to Idle Mode blocked - prop command levels indicate landing has not occurred")
        elif sw5 == 1 and sw8 == 1:
            print("Switch to Emergency Off Mode")
            flt_state = 5
            em_off = True
        elif sw6 == 2:
            print("Switch to Auto-Land Mode")
            flt_state = 2
    # From Auto-Land Mode go to either Flight or Emergency Off Mode
    elif flt_state == 2:
        if sw5 == 1 and sw8 == 1:
            print ("Switch to Emergency Off Mode")
            flt_state = 5
            em_off = True
        elif sw6 == 1:
            if umag < 100:    # control command level must be less than 100 counts
                flt_state = 1
                print("Switched to Flt Mode")
            else:
                print("Switch to Flight Mode blocked due to unsafe prop command level")
    # From temporary Emergency Off, go to Flight Mode iff prop command levels low and safe
    elif flt_state == 5:
        if sw5 == 0 or sw8 == 0:
            if umag < 100:    # control command level must be less than 100 counts
                flt_state = 1
                em_off = False
                print("Switched to Flt Mode")
            else:
                print("Switch to Flight Mode blocked due to unsafe prop command level")
 
	# -----------------------------------------------------------
    # Update telemetry plotting output data based on keyboard input commands
	# -----------------------------------------------------------
    if (char != []):
        try:
            ichar = int(char)
        except:
            ichar = 0
        # Currently the control variable 'cv' is not settable thru the keyboard but is fixed to 1
        #if (ichar > 1 and ichar < 2):
            # cv = 0 is unaided (raw input) control and can only be set from the main routine
            # cv = 1 is the normal aided (altitude and heading hold) control setting
            # cv = 2 is currently unused
            # cv = 3 is set internally to elicite a fast calibration time period
            # cv = ichar - 1
        #el
        if (ichar == 4):
            telem_flag = 0
        elif (ichar > 4 and ichar < 10):
            telem_flag = 1
            telm_idx = ichar - 4
				
    return flt_state, statics_rc, telem_flag, telm_idx, cv, RC_cal_data, em_off

