# module: dynamixel.py
# Rajan Iyengar Neurorobotics lab, UWaterloo
# This module deals with setting up of the USB to Dynamixel code needed to drive the 4 servos in Reflex_SF
# The Reflex_SF uses USB2dynamixel adaptor to control the four dynamixel MX-28 servos. The servos operate in
# multi-turn mode. The setting up of the USB to Dynamixel code is borrowed and modified from the following
# site: http://www.hizook.com/files/users/3/lib_robotis.py_.txt.
#-----------------------Acknowledgement -----------------------------------
# For the code that was obtained from http://www.hizook.com/files/users/3/lib_robotis.py_.txt:
# Copyright (c) 2009, Georgia Tech Research Corporation
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#     * Redistributions of source code must retain the above copyright
#       notice, this list of conditions and the following disclaimer.
#     * Redistributions in binary form must reproduce the above copyright
#       notice, this list of conditions and the following disclaimer in the
#       documentation and/or other materials provided with the distribution.
#     * Neither the name of the Georgia Tech Research Corporation nor the
#       names of its contributors may be used to endorse or promote products
#       derived from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY GEORGIA TECH RESEARCH CORPORATION ''AS IS'' AND
# ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
# WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL GEORGIA TECH BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
# LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,
# OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
# LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
# OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
# ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
#
# Authors: Travis Deyle, Advait Jain & Marc Killpack (Healthcare Robotics Lab, Georgia Tech.)
#---- End of Acknowledgement

import serial
import time
import thread
import sys, optparse
import math
import string

import pwd
import os
import grp



class USB2Dynamixel_Device():
    ''' Class that manages serial port contention between servos on same bus
    '''
    def __init__( self, dev_name = '/dev/ttyUSB0', baudrate = 57600 ):
        try:
            self.dev_name = string.atoi( dev_name ) # stores the serial port as 0-based integer for Windows
        except:
            self.dev_name = dev_name # stores it as a /dev-mapped string for Linux / Mac

        self.mutex = thread.allocate_lock()
        self.servo_dev = None

        self.acq_mutex()
        self._open_serial( baudrate )
        self.rel_mutex()

    def acq_mutex(self):
        self.mutex.acquire()

    def rel_mutex(self):
        self.mutex.release()

    def write_serial(self, msg):
        # It is up to the caller to acquire / release mutex
        self.servo_dev.write( msg )

    def read_serial(self, nBytes=1):
        # It is up to the caller to acquire / release mutex
        rep = self.servo_dev.read( nBytes )
        return rep

    def _open_serial(self, baudrate):

        # trying to find out why permission denied
        #        user = pwd.getpwuid(os.getuid()).pw_name
        #        print "The user is :" + user

        #        groups = [g.gr_name for g in grp.getgrall() if user in g.gr_mem]
        #        gid = pwd.getpwnam(user).pw_gid
        #        groups.append(grp.getgrgid(gid).gr_name)
        #        print "The belongs to the the groups:"
        #        print groups, "\n"

        #        print "The permission for /dev/ttyUSB0 is ", oct(os.stat("/dev/ttyUSB0").st_mode & 0777)
        #        The group is dialout and the user needs to be added to the group for permissions to work.
        #        print "If you still have errors, you may have added the user to the group later.and python is reading from DB"
        #        Login and logout for your program to have the group permission
        #
        # permission denied debugging

        try:
            self.servo_dev = serial.Serial(self.dev_name, baudrate, timeout=1.0)
            # Closing the device first seems to prevent "Access Denied" errors on WinXP
            # (Conversations with Brian Wu @ MIT on 6/23/2010)
            self.servo_dev.close()
            self.servo_dev.setParity('N')
            self.servo_dev.setStopbits(1)
            self.servo_dev.open()

            self.servo_dev.flushOutput()
            self.servo_dev.flushInput()

        except (serial.serialutil.SerialException), e:
            print e.args
            raise RuntimeError('lib_robotis: Serial port issue!\n')
        if(self.servo_dev == None):
            raise RuntimeError('lib_robotis: Serial port not found!\n')


class Robotis_Servo():
    ''' Class to use a robotis MX-28
    '''
    def __init__(self, USB2Dynamixel, servo_id, series = None ):
        ''' USB2Dynamixel - USB2Dynamixel_Device object to handle serial port.
                            Handles threadsafe operation for multiple servos
            servo_id - servo ids connected to USB2Dynamixel 1,2,3,4 ... (1 to 253)
                       [0 is broadcast if memory serves]
        '''
        # not sure how I want to use the defaults yet - 28 Nov. 2015 - Rajan
        defaults = {
            'home_encoder': 0x7FF,
            'max_encoder': 0xFFF,
            'rad_per_enc': math.radians(360.0) / 0xFFF,
            'max_ang': math.radians(180),
            'min_ang': math.radians(-180),
            'flipped': False,
            'max_speed': math.radians(5) #changed from 100 to 5 by Rajan
        }



        # Error Checking
        if USB2Dynamixel == None:
            raise RuntimeError('lib_robotis: Robotis Servo requires USB2Dynamixel!\n')
        else:
            self.dyn = USB2Dynamixel

        # ID exists on bus?
        self.servo_id = servo_id
        try:
            if (self.read_servo_id() != servo_id):
                print 'The servo-id address is not what it should be - You should never hit this line'
        except:
            raise RuntimeError('lib_robotis: Error encountered.  Could not find ID (%d) on bus (%s), or USB2Dynamixel \
                3-way switch in wrong position.\n' % ( servo_id, self.dyn.dev_name ))

        # Set Return Delay time - Used to determine when next status can be requested
        data = self.read_address( 0x05, 1)
        self.return_delay = data[0] * 2e-6
        # Set various parameters.  Load from servo_config.
        self.settings = {}

        #Set to default any parameter not specified in servo_config
        for key in defaults.keys():
            if key in self.settings: #modified to be compatible to Python 3
                pass
            else:
                self.settings[ key ] = defaults[ key ]

    def kill_cont_turn(self):
        '''resets CCW angle limits to allow commands through 'move_angle' again
        '''
        self.write_address(0x08, [255, 3])

    def is_moving(self):
        ''' returns True if servo is moving.
        '''
        data = self.read_address( 0x2e, 1 )
        return data[0] != 0

    def read_voltage(self):
        ''' returns voltage (Volts)
        '''
        data = self.read_address( 0x2a, 1 )
        return data[0] / 10.

    def read_temperature(self):
        ''' returns the temperature (Celcius)
        '''
        data = self.read_address( 0x2b, 1 )
        return data[0]

    def read_load(self):
        ''' number proportional to the torque applied by the servo.
            sign etc. might vary with how the servo is mounted.
        '''
        data = self.read_address( 0x28, 2 )
        load = data[0] + (data[1] >> 6) * 256
        if data[1] >> 2 & 1 == 0:
            return -1.0 * load
        else:
            return 1.0 * load


    def read_encoder(self):
        ''' returns position in encoder ticks
        '''
        data = self.read_address( 0x24, 2 )
        enc_val = data[0] + data[1] * 256
        return enc_val

    #Rajan's addition
    def read_servo_id(self):
        '''read the servo-id from location address 0x03 one byte
        '''
        data = self.read_address(0x03,1)
        servo_id = data[0]
        return servo_id

    def read_cw_angle_limit(self):
        ''' returns clockwise angle limits
        '''
        data = self.read_address(0x06,2)
        cwa_val = data[0] + data[1] * 256
        return cwa_val

    def read_ccw_angle_limit(self):
        ''' returns counter-clockwise angle limits
        '''
        data = self.read_address(0x08,2)
        ccwa_val = data[0] + data[1] * 256
        return ccwa_val

    def read_resolution_divider(self):
        resolution = self.read_address(0x16,1)
        return resolution

    def read_current_position(self):
        ''' returns current position
        '''
        data = self.read_address(0x24,2)
        current_position = data[0] + data[1] * 256
        return current_position

    def read_offset(self):
        '''Read Multi-turn offset
        '''
        data = self.read_address(0x14,2)
        current_offset = data[0] + data[1] * 256
        return current_offset

    def set_goal_position(self, n):
        ''' move to n
        '''
        hi,lo = n / 256, n % 256
        return self.write_address(0x1e, [lo,hi])

    def get_goal_position(self):
        ''' get goal position
        '''
        data = self.read_address(0x1e,2)
        current_goal_position = data[0] + data[1] * 256
        return current_goal_position

    def get_speed(self):
        data = self.read_address(0x20,2)
        current_speed = data[0] + data[1] * 256
        current_speed = current_speed*0.114
        return current_speed

    def set_speed(self,n):
        ''' 0 - Max speed. n should be less than 1023
        '''

        hi,lo = n / 256, n % 256
        if (hi>3):
            hi = 3
        return self.write_address(0x20, [lo,hi])

    def read_raw_load(self):
        # raw 2 byte
        data = self.read_address( 0x28, 2 )
        load = data[0] + data[1] * 256
        return load

    def read_and_convert_raw_load(self):
        # raw 2 byte
        data = self.read_address( 0x28, 2 )
        load = data[0] + data[1] * 256
        if load  <= 1023:
            rotation = "Counter Clockwise"
        else:
            rotation = "Clockwise"
            load = load - 1023
        return(load,rotation)

    def read_max_torque(self):
        data = self.read_address( 0x0E, 2 )
        max_torque = data[0] + data[1] * 256
        return max_torque

    def read_set_torque(self):
        #percentage of max_torque available during operations
        data = self.read_address( 0x22, 2 )
        set_torque = data[0] + data[1] * 256
        return set_torque

    #end of Rajan's addition

    def read_angle(self):
        ''' returns the current servo angle (radians)
        '''
        ang = (self.read_encoder() - self.settings['home_encoder']) * self.settings['rad_per_enc']
        if self.settings['flipped']:
            ang = ang * -1.0
        return ang

    def move_angle(self, ang, angvel=None, blocking=True):
        ''' move to angle (radians)
        '''
        if angvel == None:
            angvel = self.settings['max_speed']

        if angvel > self.settings['max_speed']:
            print 'lib_robotis.move_angle: angvel too high - %.2f deg/s' % (math.degrees(angvel))
            print 'lib_robotis.ignoring move command.'
            return

        if ang > self.settings['max_ang'] or ang < self.settings['min_ang']:
            print 'lib_robotis.move_angle: angle out of range- ', math.degrees(ang)
            print 'lib_robotis.ignoring move command.'
            return

        self.set_angvel(angvel)

        if self.settings['flipped']:
            ang = ang * -1.0
        enc_tics = int(round( ang / self.settings['rad_per_enc'] ))
        enc_tics += self.settings['home_encoder']
        self.move_to_encoder( enc_tics )

        if blocking == True:
            while(self.is_moving()):
                continue

    def move_to_encoder(self, n):
        ''' move to encoder position n
        '''
        # In some border cases, we can end up above/below the encoder limits.
        #   eg. int(round(math.radians( 180 ) / ( math.radians(360) / 0xFFF ))) + 0x7FF => -1
        n = min( max( n, 0 ), self.settings['max_encoder'] )
        hi,lo = n / 256, n % 256
        return self.write_address( 0x1e, [lo,hi] )

    def enable_torque(self):
        return self.write_address(0x18, [1])

    def disable_torque(self):
        return self.write_address(0x18, [0])

    def set_angvel(self, angvel):
        ''' angvel - in rad/sec
        '''
        rpm = angvel / (2 * math.pi) * 60.0
        angvel_enc = int(round( rpm / 0.111 ))
        if angvel_enc<0:
            hi,lo = abs(angvel_enc) / 256 + 4, abs(angvel_enc) % 256
        else:
            hi,lo = angvel_enc / 256, angvel_enc % 256

        return self.write_address( 0x20, [lo,hi] )

    def write_id(self, id):
        ''' changes the servo id
        '''
        return self.write_address( 0x03, [id] )

    def __calc_checksum(self, msg):
        chksum = 0
        for m in msg:
            chksum += m
        chksum = ( ~chksum ) % 256
        return chksum

    def read_address(self, address, nBytes=1):
        ''' reads nBytes from address on the servo.
            returns [n1,n2 ...] (list of parameters)
        '''
        msg = [ 0x02, address, nBytes ]
        return self.send_instruction( msg, self.servo_id )

    def write_address(self, address, data):
        ''' writes data at the address.
            data = [n1,n2 ...] list of numbers.
            return [n1,n2 ...] (list of return parameters)
        '''
        msg = [ 0x03, address ] + data
        return self.send_instruction( msg, self.servo_id )

    def send_instruction(self, instruction, id):
        msg = [ id, len(instruction) + 1 ] + instruction # instruction includes the command (1 byte + parameters. length = parameters+2)
        chksum = self.__calc_checksum( msg )
        msg = [ 0xff, 0xff ] + msg + [chksum]

        self.dyn.acq_mutex()
        try:
            self.send_serial( msg )
            data, err = self.receive_reply()
        except:
            self.dyn.rel_mutex()
            raise
        self.dyn.rel_mutex()

        if err != 0:
            self.process_err( err )

        return data

    def process_err( self, err ):
        raise RuntimeError('lib_robotis: An error occurred: %d\n' % err)

    def receive_reply(self):
        start = self.dyn.read_serial( 2 )
        if start != '\xff\xff':
            raise RuntimeError('lib_robotis: Failed to receive start bytes\n')
        servo_id = self.dyn.read_serial( 1 )
        if ord(servo_id) != self.servo_id:
            raise RuntimeError('lib_robotis: Incorrect servo ID received: %d\n' % ord(servo_id))
        data_len = self.dyn.read_serial( 1 )
        err = self.dyn.read_serial( 1 )
        data = self.dyn.read_serial( ord(data_len) - 2 )
        checksum = self.dyn.read_serial( 1 ) # I'm not going to check...
        return [ord(v) for v in data], ord(err)


    def send_serial(self, msg):
        """ sends the command to the servo
        """
        out = ''
        for m in msg:
            out += chr(m)
        self.dyn.write_serial( out )







