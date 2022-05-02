
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
    along with this program.  If not, see https://www.gnu.org/licenses/.
    
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
import socket
import string
import fcntl
import tty
import termios
import getopt
import math
from array import *
import select
import smbus
import select
import struct
import subprocess
from datetime import datetime
import shutil
import pdb

import sys
import time
import os
import sys

# ---------------------------------------------------------------------
# The keyPressed function determines which key has been pressed by
# accessing the system files.  If pressed it returns the pressed key
# as a variable.
# ---------------------------------------------------------------------
def keyPressed():
    fd = sys.stdin.fileno()
    oldterm = termios.tcgetattr(fd)
    newattr = termios.tcgetattr(fd)
    newattr[3] = newattr[3] & ~termios.ICANON & ~termios.ECHO
    termios.tcsetattr(fd, termios.TCSANOW, newattr)
    oldflags = fcntl.fcntl(fd, fcntl.F_GETFL)
    fcntl.fcntl(fd, fcntl.F_SETFL, oldflags | os.O_NONBLOCK)
    try:
        while True:
            try:
                c = sys.stdin.read(1)
                return c
            except IOError:
                return False
    finally:
        termios.tcsetattr(fd, termios.TCSAFLUSH, oldterm)
        fcntl.fcntl(fd, fcntl.F_SETFL, oldflags)

# ---------------------------------------------------------------------
# The kbcmd function allows setpoint inputs to be injected using the
# keyboard rather than the radio controller.  It is currently not being 
# used.
# ---------------------------------------------------------------------
# Inputs:
#   flt_state -- flight state machine index (0 - idle, 1 - flight, 2 - calibrate, 3 - landed)
#   the_p_trim -- pitch trim angle (deg)
#   the_r_trim -- roll trim angle (deg)
#   omeg_y_trim -- yaw angular rate trim (deg/sec)
#   vthrust_trim -- vertical thrust trim (ESC counts)
# Outputs:
#   vthrust_sp -- vertical thrust setpoint (ESC counts)
#   thet_pitch_sp -- pitch angle setpoint (rad)
#   thet_roll_sp -- roll angle setpoint (rad)
#   omeg_yaw_sp -- yaw angular rate setpoint (rad/sec)
# Constants: 
#   d_thet -- roll/pitch trim angle increment 
#   t_thet -- roll/pitch setpoint temporal offset
#   d_omega -- yaw rate trim increment
#   t_omega -- yaw rate setpoint temporal offset
#   D_omega -- yaw rate trim large increment
#   d_thrust -- thrust trim increment
#   t_thrust -- thrust setpoint temporal offset
#   D_thrust -- thrust trim large increment

def kbcmd(flt_state, the_p_trim, the_r_trim, omeg_y_trim, vthrust_trim):

    # Trim adjust constants
    d_thet = 0.2   # deg
    D_thet = 1.0   # deg
    d_omega = 1.0  # deg/sec
    D_omega = 2.5  # deg/sec
    d_thrust = 5   # counts
    D_thrust = 50  # counts

    t_thet = 5.0   # deg
    t_omega = 45.0 # deg/sec
    t_thrust = 50  # counts

    thet_pitch_sp = 0
    thet_roll_sp = 0
    omeg_yaw_sp = 0	
    vthrust_sp = 0

    char = keyPressed()

    # Flight Command Setpoints ************************
    if flt_state == 1:
        # Respond to trim input adjustment commands
        # lower case - small increments
        # upper case - large increments

        # THRUST
        if (char == "e"):
            vthrust_trim += d_thrust
        elif (char == "d"):
            vthrust_trim -= d_thrust            
        if (char == "E"):
            vthrust_trim += D_thrust
        elif (char == "D"):
            vthrust_trim -= D_thrust

        # YAW
        if (char == "x"):
            omeg_y_trim -= d_omega
        elif (char == "c"):
            omeg_y_trim += d_omega
        if (char == "X"):
            omeg_y_trim -= D_omega
        elif (char == "C"):
            omeg_y_trim += D_omega
		
        # PITCH
        if (char == "o"):
            the_p_trim -= d_thet
        elif (char == "l"):
            the_p_trim += d_thet
        if (char == "O"):
            the_p_trim -= D_thet
        elif (char == "L"):
            the_p_trim += D_thet

        # ROLL
        if (char == ","):
            the_r_trim -= d_thet
        elif (char == "."):
            the_r_trim += d_thet		
        if (char == "<"):
            the_r_trim -= D_thet
        elif (char == ">"):
            the_r_trim += D_thet

        # Respond to temporal surge input commands
        vthrust_temp = 0
        pitch_temp = 0
        roll_temp = 0
        omeg_y_temp = 0

        # THRUST
        if (char == "t"):
            vthrust_temp = t_thrust
        elif (char == "g"):
            vthrust_temp = -t_thrust            

        # YAW
        if (char == "v"):
            omeg_y_temp = -t_omega
        elif (char == "b"):
            omeg_y_temp = +t_omega

        # PITCH
        if (char == "u"):
            pitch_temp = -t_thet
        elif (char == "j"):
            pitch_temp = t_thet

        # ROLL            
        if (char == "n"):
            roll_temp = -t_thet
        elif (char == "m"):
            roll_temp = t_thet	

        # sum the trim and temporal surge input commands
        vthrust_sp = vthrust_trim + vthrust_temp
        thet_pitch_sp = the_p_trim + pitch_temp	
        thet_roll_sp = the_r_trim + roll_temp
        omeg_yaw_sp = omeg_y_trim + omeg_y_temp

    if char and (flt_state == 1):
        print("Setpoint: vt=" + str(vthrust_sp) + "  th_p=" + str(thet_pitch_sp) + "  th_r=" + str(thet_roll_sp) + "  om_y=" + str(omeg_yaw_sp))

    #Convert from deg to rad
    omeg_yaw_sp   /= 57.3
    thet_pitch_sp /= 57.3
    thet_roll_sp  /= 57.3

    return vthrust_sp, thet_pitch_sp, thet_roll_sp, omeg_yaw_sp, the_p_trim, the_r_trim, omeg_y_trim, vthrust_trim, char

