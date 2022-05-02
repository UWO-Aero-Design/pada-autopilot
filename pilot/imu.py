
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

from RPIO import PWM

CHANNEL = 0

class ESC:
	pwm = None
        
	def __init__(self, pin):  
		# Set pin number for this ESC
		self.esc_pin = pin
		# PWM pulse width in microseconds
		self.min_pulse = 1000
		self.max_pulse = 2000
		#  PWM pulse range
		self.next_pulse = self.min_pulse
		# Initialize the RPIO PWM
		if not PWM.is_setup():
			PWM.set_loglevel(PWM.LOG_LEVEL_ERRORS)     
			#PWM.set_loglevel(PWM.LOG_LEVEL_DEBUG)    #### this is a test line
			PWM.setup(1)    # 1usec resolution
			PWM.init_channel(CHANNEL, 3000) # 3msec sample period (the minimum)
		PWM.add_channel_pulse(CHANNEL, self.esc_pin, 0, self.next_pulse)

        
	def update(self, cmd_width):
		self.next_pulse = int(self.min_pulse + cmd_width)
		if self.next_pulse < self.min_pulse:
			self.next_pulse = self.min_pulse
		if self.next_pulse > self.max_pulse:
			self.next_pulse = self.max_pulse
		PWM.add_channel_pulse(CHANNEL, self.esc_pin, 0, self.next_pulse)

