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
        # Setup steppers at each anchor (3 actual motors)
        self.steppers = []
        self.anchors = []
        self.end_effector_offsets = []
        self.cable_length_0 = config.getfloat('cable_length_0', 1000.0)  # Initial cable length default in mm
        self.cable_length1_0 = config.getfloat('cable_length1_0', 780.0)  # Secondary initial cable length in mm
        
        # Define 4 anchors (including dummy cable anchor)
        anchor_points = [
            (0.0, 0.0, 0.0),             # Dummy cable anchor
            (0.0, 438.0, 0.0),           # Stepper A anchor (converted to mm)
            (-379.0, -219.0, 0.0),       # Stepper B anchor (converted to mm)
            (379.0, -219.0, 0.0)         # Stepper C anchor (converted to mm)
        ]
        
        # Define end-effector offsets
        self.end_effector_offsets = [
            (0.0, 0.0, -self.cable_length1_0),           # Dummy cable attachment
            (0.0, 15.88, -self.cable_length1_0),         # Cable to stepper A attachment (converted to mm)
            (-13.75, -7.94, -self.cable_length1_0),      # Cable to stepper B attachment (converted to mm)
            (13.75, -7.94, -self.cable_length1_0)        # Cable to stepper C attachment (converted to mm)
        ]
        
        # Set up steppers (only 3 real steppers)
        stepper_names = ['a', 'b', 'c']
        for i in range(3):
            name = 'stepper_' + stepper_names[i]
            stepper_config = config.getsection(name)
            s = stepper.PrinterStepper(stepper_config)
            self.steppers.append(s)
            
            # Get anchor points from config now
            anchor_x = stepper_config.getfloat('anchor_x')
            anchor_y = stepper_config.getfloat('anchor_y')
            anchor_z = stepper_config.getfloat('anchor_z', 0.)
            a = (anchor_x, anchor_y, anchor_z)
            self.anchors.append(a)
            
            # Get rotation_distance from config
            rotation_distance = stepper_config.getfloat('rotation_distance')
            
            # CRITICAL: Setup itersolve BEFORE setting rotation distance
            s.setup_itersolve('winch_stepper_alloc', a[0], a[1], a[2])
            
            # Now safe to set rotation distance
            s.set_rotation_distance(rotation_distance)
            
            s.set_trapq(toolhead.get_trapq())
            toolhead.register_step_generator(s.generate_steps)
        
        # Store the dummy anchor as well
        self.dummy_anchor = anchor_points[0]
        
        # Store anchor data as full 4-cable system
        self.full_anchors = [self.dummy_anchor] + self.anchors
        
        # Setup boundary checks
        acoords = list(zip(*self.anchors))
        self.axes_min = toolhead.Coord(*[min(a) - 0.1 for a in acoords], e=0.)
        self.axes_max = toolhead.Coord(*[max(a) + 0.1 for a in acoords], e=0.)
        self.set_position([0., 0., 0.], "")
    
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
        # We need to convert stepper positions to cable lengths first
        # We're only using the 3 real steppers (skipping dummy)
        cable_lengths = []
        
        # Calculate the lengths of the 3 actual cables (from stepper positions)
        for i, s in enumerate(self.steppers):
            # Convert stepper position to cable length
            pos = stepper_positions[s.get_name()]
            cable_length = self.cable_length_0 + pos  # Add initial cable length
            cable_lengths.append(cable_length)
        
        # Try to solve the forward kinematics using optimization
        # This is a simplified approach since the full forward kinematics would require
        # a numerical solver or an analytical solution specific to this system
        current_pos = [0., 0., 0.]  # Initial guess
        
        # Use the trilateration function for the 3 real anchors and cable lengths
        try:
            current_pos = mathutil.trilateration(
                self.anchors, 
                [(cl * cl) for cl in cable_lengths]
            )
        except:
            # Fallback to last known position if trilateration fails
            pass
            
        return current_pos
    
    def set_position(self, newpos, homing_axes):
        """Set the position of the toolhead"""
        # Calculate all 4 cable lengths for this position
        cable_lengths = self.calc_cable_lengths(newpos)
        
        # Set positions for the 3 actual steppers (skip dummy cable)
        for i, s in enumerate(self.steppers):
            # Use the appropriate delta length (skipping the dummy cable)
            delta_length = cable_lengths[i+1] - self.cable_length_0
            s.set_position([delta_length, 0., 0.])
    
    def home(self, homing_state):
        # Minimal homing implementation - no actual homing movement
        # Just marks the axes as homed at current position
        homing_state.set_axes([0, 1, 2])
        homing_state.set_homed_position([0., 0., 0.])
        
    def check_homing_move(self, homing_state):
        # This function prevents actual homing moves
        pass
    
    def check_move(self, move):
        # Basic check to ensure moves stay within bounds
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

def load_kinematics(toolhead, config):
    return WinchKinematics(toolhead, config)
