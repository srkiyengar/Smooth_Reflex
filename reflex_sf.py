__author__ = 'srkiyengar'



import dynamixel
import pygame
import joystick
from datetime import datetime
import logging
import logging.handlers

import serial
import time
import thread
import sys, optparse
import math
import string

import pwd
import os
import grp

JOY_DEADZONE_A0 = 0.2
JOY_DEADZONE_A1 = 0.04

SCAN_RATE = 5           #1 second divided by scan rate is t he joystick scanning
POS_ERROR = 200

MOVE_TICKS = 40


MAX_FINGER_MOVEMENT = 2300
MAX_SPEED = 1000 # A max speed of 1023 is allowed
LOG_LEVEL = logging.INFO
LOG_FILENAME = 'smooth_reflex_movement' + datetime.now().strftime('%Y-%m-%d---%H:%M:%S')

class reflex_sf():
    '''The class manages the calibration and movement of the fingers for pinch and grasp
    '''
    def __init__(self, usb_channel = '/dev/ttyUSB0', baudrate = 57600):
        dyn = dynamixel.USB2Dynamixel_Device(usb_channel, baudrate)
        l_limits = [0,14000,16600,15700, 16384]
        u_limits = [0,0,0,0,0]
        self.finger = []
        self.finger.append(0) # finger starts with 1. Inserting 0 at the first list position
        for i in range(1,5,1):
            try:
                # using the USB2Dynamixel object try to send commands to each and receive information
                j= dynamixel.Robotis_Servo(dyn, i,"MX" )
            except:
                raise RuntimeError('Connection to Servo failure for servo number', i,'\n')
            temp = j.read_temperature()
            resol = j.read_resolution_divider()
            current_pos = j.read_current_position()
            goal_pos = j.get_goal_position()
            offset = j.read_offset()
            speed = MAX_SPEED
            j.set_speed(speed)
            if i==1:
                joint_state = 1
                u_limits[i] = l_limits[i] + MAX_FINGER_MOVEMENT
            elif i==2:
                joint_state = -1
                u_limits[i] = l_limits[i] - MAX_FINGER_MOVEMENT
            elif i==3:
                joint_state = 1
                u_limits[i] = l_limits[i] + MAX_FINGER_MOVEMENT
            elif i==4:
                joint_state = -1
                u_limits[i] = l_limits[i] - MAX_FINGER_MOVEMENT

            max_torque = j.read_max_torque()
            set_torque = j.read_set_torque()
            finger_parameters = {"servo":j, "temperature": temp, "resolution_divider": resol, "initial_position": current_pos,
                                 "goal_position":goal_pos,"multi_turn_offset":offset, "moving_speed":speed, "direction": 1,
                                 "lower_limit":l_limits[i],"upper_limit":u_limits[i],"rotation":joint_state,
                                 "max_torque":max_torque, "set_torque":set_torque, "GP":current_pos}
            self.finger.append(finger_parameters)



    def is_finger_within_limit(self, id, new_position):
        ll = self.finger[id]["lower_limit"]
        ul = self.finger[id]["upper_limit"]
        rotation_mode = self.finger[id]["rotation"]
        if rotation_mode == 1:
            if ul >= new_position >= ll:
                return new_position
            else:
                if new_position > ul:
                    new_position = ul
                elif new_position < ll:
                    new_position = ll
                my_logger.debug('Finger {} new position changed to {}'.format(id,new_position))
                return new_position
        elif rotation_mode == -1:
           if ll>=new_position >= ul:
               return new_position
           else:
               if new_position > ll:
                    new_position = ll
               elif new_position < ul:
                    new_position = ul
               my_logger.debug('Finger {} new position changed to {}'.format(id,new_position))
               return new_position
        else:
            my_logger.debug("Finger{} joint rotation mode: {} unknown",format(rotation_mode))
            return 0

    def finger_current_position(self,id):
        while (self.finger[id]["servo"].is_moving()):
            pass
        p = self.finger[id]["servo"].read_current_position()
        my_logger.debug('Finger{} - Current Position {}'.format(id,p))
        return p

    def finger_load(self,id):
        load, rotation = self.finger[id]["servo"].read_and_convert_raw_load()
        return load, rotation

    def move_finger_delta(self, id, move_direction,increment): # direction +1 = finger closing; -1 = finger opening
        p = self.finger[id]["GP"]
        my_logger.debug('Before - Finger {} - GP {}'.format(id,p))
        q = self.finger[id]["rotation"]
        q *= move_direction
        new_position = p + q*increment
        move_to = self.is_finger_within_limit(id,new_position)
        my_logger.debug('After - Finger {} - GP {}'.format(id,new_position))
        if move_to > 0:
            my_logger.info("Wanted to move to {} moving to {}".format(new_position,move_to))
            my_logger.info('Finger {} - Moving From Position {} to Position {}'.format(id,p,move_to))
            self.finger[id]["servo"].set_goal_position(move_to) # return data to make the program wait
            self.finger[id]["GP"] = move_to     # new_position when out of bounds will be modified. Therefore
            return move_to
        else:
            my_logger.info\
                ('Outside Limit Finger{} - Denied: Move From Position {} to Position {}'.format(id,p,new_position))
            return 0

    def grip_fingers(self, move_by, grip):
        if grip == 1:
            my_logger.info('Tighten by {} '.format(move_by))
        elif grip == -1:
            my_logger.info('Loosen by {} '.format(move_by))
        self.move_finger_delta(1,grip,move_by)
        self.move_finger_delta(2,grip,move_by)
        self.move_finger_delta(3,grip,move_by)

    def space_finger1_and_finger2(self, move_by, grip):
        if grip == 1:
            my_logger.info('Spread finger 1 and 2 by {}'.format(move_by))
        elif grip == -1:
            my_logger.info('Bring finger 1 and 3 closer by {}'.format(move_by))
        self.move_finger_delta(4,grip,move_by)



# Define some colors
BLACK    = (   0,   0,   0)
WHITE    = ( 255, 255, 255)

# This is a simple class that will help us print to the screen
# It has nothing to do with the joysticks, just outputing the
# information.
class TextPrint:
    def __init__(self):
        self.reset()
        self.font = pygame.font.Font(None, 20)

    def Screenprint(self, screen, textString):
        textBitmap = self.font.render(textString, True, BLACK)
        screen.blit(textBitmap, [self.x, self.y])
        self.y += self.line_height

    def reset(self):
        self.x = 10
        self.y = 10
        self.line_height = 15

    def indent(self):
        self.x += 10

    def unindent(self):
        self.x -= 10

    def Yspace(self):
        self.y += 10


if __name__ == '__main__':



    # Set up a logger with output level set to debug; Add the handler to the logger
    my_logger = logging.getLogger('MyLogger')
    my_logger.setLevel(LOG_LEVEL)
    handler = logging.handlers.RotatingFileHandler(LOG_FILENAME, maxBytes=2000000, backupCount=5)
    formatter = logging.Formatter('%(asctime)s - %(name)s - %(levelname)s - %(message)s')
    handler.setFormatter(formatter)
    my_logger.addHandler(handler)
    # end of logfile preparation Log levels are debug, info, warn, error, critical


    palm = reflex_sf() # Reflex object ready

    my_logger.info('Reflex_SF object created')



    for i in range(1,5,1):
        lowest_position = palm.finger[i]["lower_limit"]
        highest_position = palm.finger[i]["upper_limit"]
        init_position = palm.finger[i]["initial_position"]
        max_torque_setting = palm.finger[i]["max_torque"]
        allowable_torque = palm.finger[i]["set_torque"]

        my_logger.info('--- Finger {}:'.format(i))
        my_logger.info('       Max Torque --- {}'.format(max_torque_setting))
        my_logger.info('       Allowable Torque --- {}'.format(allowable_torque))
        my_logger.info('       Lower Limit Position --- {}'.format(lowest_position))
        my_logger.info('       Upper Limit Position --- {}'.format(highest_position))
        my_logger.info('       Initial Position {}'.format(init_position))

        if (i == 1 or i == 3):
            a = lowest_position - POS_ERROR
            b= highest_position + POS_ERROR
            if a >= init_position or init_position >= b:
                my_logger.info('Servo {} Initial Position {} not between Lower Limit {} and Upper Limit {}'\
                               .format(i,init_position,lowest_position,highest_position))
                raise RuntimeError(
                'Servo %d Initial Position %d not between Lower Limit %d and Upper Limit %d',\
                i,init_position,lowest_position,highest_position)
        elif (i == 2):
            a = lowest_position + POS_ERROR
            b = highest_position - POS_ERROR
            if a <= init_position or init_position <= b:
                my_logger.info('Servo {} Initial Position {} not between Lower Limit {} and Upper Limit {}'\
                               .format(i,init_position,lowest_position,highest_position))
                raise RuntimeError(
                'Servo %d Initial Position %d not between Lower Limit %d and Upper Limit %d',\
                i,init_position,lowest_position,highest_position)

    pygame.init()

    # Set the width and height of the screen [width,height]
    size = [500, 700]
    screen = pygame.display.set_mode(size)

    pygame.display.set_caption("Reflex_SF Commands")

    # Used to manage how fast the screen updates
    clock = pygame.time.Clock()

    # Get ready to print
    textPrint = TextPrint()

    j_device = joystick.ExtremeProJoystick()
    # Get count of joystick
    Buttons = []
    Num_Buttons = j_device.buttons
    Axes = []   #mainly for the screen display
    min_val = [-JOY_DEADZONE_A0,-JOY_DEADZONE_A1,0,0]
    max_val = [JOY_DEADZONE_A0,JOY_DEADZONE_A1,0,0]

    Num_Axes = j_device.axes
    Num_Hats =j_device.hats



    for i in range (Num_Buttons):
        Buttons.append(0)

    for i in range (Num_Axes):
        Axes.append(0.00)


    move_goal = [0,0,0,0]
    Hat = (0,0)

    #Loop until the user clicks the close button.
    done = False

    # -------- Main Program Loop -----------
    while done==False:
        screen.fill(WHITE)
        textPrint.reset()
        for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    done = True
                elif event.type == pygame.JOYAXISMOTION:
                    position = event.dict['value']
                    k = event.dict['axis']
                    Axes[k] = position
                    if position > 0:
                        if position > max_val[k]:
                            direction = 1
                            move_goal[k] = int(position*MOVE_TICKS)
                            if k == 1:
                                my_logger.info("@--Joy Axis {} +ive Value {},moveby {}".format(k,position,move_goal[k]))
                                palm.grip_fingers(move_goal[k],direction)
                            elif k==0:
                                my_logger.info("@--Joy Axis {} +ive Value {},moveby {}".format(k,position,move_goal[k]))
                                palm.space_finger1_and_finger2(move_goal[k],direction)
                    elif position < 0:
                        if position < min_val[k]:
                            direction = -1
                            move_goal[k] = int(abs(position)*MOVE_TICKS)
                            if k == 1:
                                my_logger.info("@--Joy Axis {} -ive Value {},moveby {}".format(k,position,move_goal[k]))
                                palm.grip_fingers(move_goal[k],direction)
                            elif k == 0:
                                my_logger.info("@--Joy Axis {} -ive Value {},moveby {}".format(k,position,move_goal[k]))
                                palm.space_finger1_and_finger2(move_goal[k],direction)
                    else:
                        pass
                elif event.type == pygame.JOYBUTTONDOWN:
                    i = event.dict['button']    # button number
                    Buttons[i] = 1
                    my_logger.debug("Button {} pressed".format(i))
                elif event.type == pygame.JOYBUTTONUP:
                    i = event.dict['button']
                    Buttons[i] = 0
                    my_logger.debug("Button {} released".format(i))
                elif event.type == pygame.JOYHATMOTION:
                    Hat = event.dict['value']
                    my_logger.debug("Hat value: {}".format(str(Hat)))
                else:
                    pass # ignoring other event types
        textPrint.Screenprint(screen, "Joystick name: {}".format(j_device.name))
        textPrint.Yspace()
        textPrint.Screenprint(screen, "Number of Axes: {}".format(Num_Axes))
        textPrint.indent()
        for i in range(Num_Axes):
            textPrint.Screenprint(screen, "Axis {} value: {:>6.3f}".format(i, Axes[i]))
        textPrint.unindent()
        textPrint.Yspace()
        textPrint.Screenprint(screen, "Number of Buttons: {}".format(Num_Buttons))
        textPrint.indent()
        for i in range(Num_Buttons):
            textPrint.Screenprint(screen, "Button {:>2} value: {}".format(i,Buttons[i]))
        textPrint.unindent()
        textPrint.Yspace()
        textPrint.Screenprint(screen, "Number of Hats: {}".format(Num_Hats) )
        textPrint.indent()
        textPrint.Screenprint(screen, "Hat value: {}".format(str(Hat)) )
        textPrint.unindent()




    # ALL CODE TO DRAW SHOULD GO ABOVE THIS COMMENT

    # Go ahead and update the screen with what we've drawn.
        pygame.display.flip()

    # Limit to 20 frames per second OR 50 ms scan rate - 1000/20 = 50 ms Both display and checking of Joystick;
        clock.tick(SCAN_RATE)

# Close the window and quit.
# If you forget this line, the program will 'hang' on exit if running from IDLE.

pygame.quit ()






