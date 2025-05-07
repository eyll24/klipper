# Code for handling the kinematics of cable winch robots with 4-cable structure (1 dummy cable)
#
# Copyright (C) 2018-2021  Kevin O'Connor <kevin@koconnor.net>
# Modified for 4-cable system with dummy first cable
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import stepper, mathutil
import math

class WinchKinematics:
    def __init__(self, toolhead, config):
        self.printer = config.get_printer()
        # Setup steppers at each anchor (3 actual motors)
        self.steppers = []
        self.anchors = []
        self.end_effector_offsets = []
        
        # Get config parameters
        self.cable_length_0 = config.getfloat('cable_length_0', 1000.0)  # Initial cable length default in mm
        self.cable_length1_0 = config.getfloat('cable_length1_0', 780.0)  # Secondary initial cable length in mm
        
        # Define end-effector offsets
        self.end_effector_offsets = [
            (0.0, 0.0, -self.cable_length1_0),           # Dummy cable attachment
            (0.0, 15.88, -self.cable_length1_0),         # Cable to stepper A attachment (converted to mm)
            (-13.75, -7.94, -self.cable_length1_0),      # Cable to stepper B attachment (converted to mm)
            (13.75, -7.94, -self.cable_length1_0)        # Cable to stepper C attachment (converted to mm)
        ]
        
        # Set up steppers (only 3 real steppers)
        self.rails = []
        for i, name in enumerate(['a', 'b', 'c']):
            rail = WinchRail(self.printer, config, name)
            self.rails.append(rail)
            self.steppers.extend(rail.get_steppers())
            self.anchors.append(rail.get_anchor())
            rail.setup_itersolve()
            rail.set_trapq(toolhead.get_trapq())
            toolhead.register_step_generator(rail.generate_steps)
        
        # Store anchor data as full 4-cable system
        self.dummy_anchor = (0.0, 0.0, 0.0)  # Dummy cable anchor
        self.full_anchors = [self.dummy_anchor] + self.anchors
        
        # Setup boundary checks
        acoords = list(zip(*self.anchors))
        self.axes_min = toolhead.Coord(*[min(a) - 0.1 for a in acoords], e=0.)
        self.axes_max = toolhead.Coord(*[max(a) + 0.1 for a in acoords], e=0.)
        self.set_position([0., 0., 0.], ())
    
    def get_steppers(self):
        return list(self.steppers)
    
    def calc_cable_lengths(self, pos):
        """Calculate all 4 cable lengths for a given position"""
        x, y, z = pos
        cable_lengths = []
        
        for i in range(4):
            if i == 0:
                # Dummy cable - use constant length
                cable_lengths.append(self.cable_length_0)
                continue
                
            bx, by, bz = self.full_anchors[i]
            qx, qy, qz = self.end_effector_offsets[i]
            
            # Cable vectors
            lx = x - bx + qx
            ly = y - by + qy
            lz = z - bz + qz
            
            length = math.sqrt(lx*lx + ly*ly + lz*lz)
            cable_lengths.append(length)
        
        return cable_lengths
    
    def calc_position(self, stepper_positions):
        """Calculate cartesian position from stepper positions"""
        # This is only called during m114 and similar reporting
        cable_lengths = []
        for i, rail in enumerate(self.rails):
            rail_pos = rail.get_commanded_position()
            cable_length = self.cable_length_0 + rail_pos
            cable_lengths.append(cable_length)
        
        try:
            current_pos = mathutil.trilateration(
                self.anchors, 
                [(cl * cl) for cl in cable_lengths]
            )
            return current_pos
        except:
            # Fallback to origin if trilateration fails
            return [0., 0., 0.]
    
    def set_position(self, newpos, homing_axes):
        """Set the position of the toolhead"""
        # Calculate all 4 cable lengths for this position
        cable_lengths = self.calc_cable_lengths(newpos)
        
        # Set positions for the 3 actual rails (skip dummy cable)
        for i, rail in enumerate(self.rails):
            delta_length = cable_lengths[i+1] - self.cable_length_0
            rail.set_position(delta_length)
    
    def home(self, homing_state):
        # Minimal homing implementation - no actual homing movement
        # Just marks the axes as homed at current position
        homing_state.set_axes([0, 1, 2])
        homing_state.set_homed_position([0., 0., 0.])
        
    def check_home_position(self, homing_state):
        # Prevent any homing moves
        return True
    
    def check_move(self, move):
        """Verify move stays in valid range"""
        end_pos = move.end_pos
        for i in range(3):
            if end_pos[i] < self.axes_min[i] or end_pos[i] > self.axes_max[i]:
                raise self.printer.command_error(
                    "Move out of bounds: %.3f %.3f %.3f" % 
                    (end_pos[0], end_pos[1], end_pos[2]))
    
    def get_status(self, eventtime):
        return {
            'homed_axes': 'xyz',
            'axis_minimum': self.axes_min,
            'axis_maximum': self.axes_max,
        }

# Helper class for single cable/rail
class WinchRail:
    def __init__(self, printer, config, name):
        self.printer = printer
        # Get stepper config
        section = 'stepper_' + name
        stepper_config = config.getsection(section)
        # Get anchor position
        self.anchor_x = stepper_config.getfloat('anchor_x')
        self.anchor_y = stepper_config.getfloat('anchor_y')
        self.anchor_z = stepper_config.getfloat('anchor_z', 0.)
        self.anchor = (self.anchor_x, self.anchor_y, self.anchor_z)
        # Create stepper
        self.stepper = stepper.PrinterStepper(stepper_config)
        
    def get_anchor(self):
        return self.anchor
        
    def get_steppers(self):
        return [self.stepper]
    
    def setup_itersolve(self):
        self.stepper.setup_itersolve('winch_stepper_alloc', 
                                    self.anchor_x, self.anchor_y, self.anchor_z)
    
    def set_trapq(self, trapq):
        self.stepper.set_trapq(trapq)
    
    def generate_steps(self, start_time, move_time):
        return self.stepper.generate_steps(start_time, move_time)
    
    def set_position(self, pos):
        # Convert to array format for stepper API
        self.stepper.set_position([pos, 0., 0.])
    
    def get_commanded_position(self):
        return self.stepper.get_commanded_position()

def load_kinematics(toolhead, config):
    return WinchKinematics(toolhead, config)
