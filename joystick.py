__author__ = 'srkiyengar'
# Significant joystick code or the basis of it comes from pygame.joystick sample code
import pygame


# Before invoking this class pygame.init() needs to be called
# calling methods would require an event call like get or pump
# The looping has to be in the external invocation
# End should have pygame quit

class ExtremeProJoystick():
    def __init__( self):
        # Initialize the joysticks
        pygame.joystick.init()
        self.joystick_count = pygame.joystick.get_count()
        found = 0
        for i in range(self.joystick_count):
            My_joystick = pygame.joystick.Joystick(i)
            name = My_joystick.get_name()
            if ("Logitech Extreme 3D" in name):
                self.name = name
                My_joystick.init()
                self.axes = My_joystick.get_numaxes()
                self.buttons = My_joystick.get_numbuttons()
                self.hats = My_joystick.get_numhats()
                self.joystick= My_joystick
                found = 1
                break
        if(found==0):
            raise RuntimeError('Logitech Extreme #D Joystick not found\n')

