# Code for handling the kinematics of Cable-Driven Parallel Robot (CDPR)
#
# Based on delta.py but customized for CDPR systems
# Inspired by ters_kinematik_matlab function

import math, logging
import stepper, pins

class CDPRKinematics:
    def __init__(self, toolhead, config):
        # Setup stepper motors for cables
        stepper_configs = [config.getsection('stepper_' + str(i)) for i in range(1, 5)]
        
        # Init rails (one for each cable)
        self.rails = []
        for i, sc in enumerate(stepper_configs):
            rail = stepper.LookupMultiRail(sc, need_position_minmax=False)
            self.rails.append(rail)
            
        # Setup max velocity and acceleration
        self.max_velocity, self.max_accel = toolhead.get_max_velocity()
        self.max_z_velocity = config.getfloat(
            'max_z_velocity', self.max_velocity,
            above=0., maxval=self.max_velocity)
        self.max_z_accel = config.getfloat('max_z_accel', self.max_accel,
                                           above=0., maxval=self.max_accel)
        
        # Cable parameters
        self.cable_length = config.getfloat('cable_length', 1.0)
        self.drum_radius = config.getfloat('drum_radius', 31.96)  # mm
        self.steps_per_rotation = config.getfloat('steps_per_rotation', 200)
        self.conversion = self.steps_per_rotation / (2 * math.pi * self.drum_radius)
        
        # Exit points of cables in global frame (x, y, z)
        self.tower_x = config.getlist('tower_x', 
                                    [0.0, 0.0, -0.379, 0.379], 
                                    count=4)
        self.tower_y = config.getlist('tower_y', 
                                    [0.0, 0.438, -0.219, -0.219], 
                                    count=4)
        self.tower_z = config.getlist('tower_z', 
                                    [0.0, 0.0, 0.0, 0.0], 
                                    count=4)
        
        # End effector attachment points in local frame
        self.effector_x = config.getlist('effector_x', 
                                       [0.0, 0.0, -0.01375, 0.01375], 
                                       count=4)
        self.effector_y = config.getlist('effector_y', 
                                       [0.0, 0.01588, -0.00794, -0.00794], 
                                       count=4)
        
        # Register steppers with the toolhead
        for r in self.rails:
            for s in r.get_steppers():
                s.set_trapq(toolhead.get_trapq())
                toolhead.register_step_generator(s.generate_steps)
        
        # Setup boundary checks
        self.need_home = True
        
        # Define workspace limits - these should be configured properly for your specific setup
        max_xy = config.getfloat('max_xy', 0.3)  # default 300mm radius
        min_z = config.getfloat('min_z', 0.0)
        max_z = config.getfloat('max_z', 0.5)  # default 500mm height
        
        self.axes_min = toolhead.Coord(-max_xy, -max_xy, min_z, 0.)
        self.axes_max = toolhead.Coord(max_xy, max_xy, max_z, 0.)
        
        # Home position (center of workspace)
        self.home_position = [0., 0., 0.2]  # 200mm above origin
        
        # Setup stepper positions
        self.set_position([0., 0., 0.], "")
        
    def get_steppers(self):
        return [s for rail in self.rails for s in rail.get_steppers()]
        
    def calc_cable_lengths(self, pos):
        """Calculate cable lengths for a given position"""
        x, y, z = pos
        cable_lengths = []
        motor_steps = []
        
        for i in range(4):
            # Calculate cable vector components (current position - tower position)
            lx = x - self.tower_x[i] + self.effector_x[i] * math.cos(0)  # Assuming rotation is 0
            ly = y - self.tower_y[i] + self.effector_y[i] * math.cos(0)  # Assuming rotation is 0
            lz = z - self.tower_z[i]
            
            # Calculate cable length
            length = math.sqrt(lx**2 + ly**2 + lz**2)
            cable_lengths.append(length)
            
            # Convert to motor steps
            steps = (length - self.cable_length) * self.conversion
            motor_steps.append(steps)
            
        return cable_lengths, motor_steps
        
    def calc_position(self, stepper_positions):
        """This would normally calculate cartesian position from stepper positions,
        but for CDPR this is complex (needs iterative solution) and not
        implemented for basic testing"""
        # For testing, just return home position
        return list(self.home_position)
        
    def set_position(self, newpos, homing_axes):
        """Set the current position"""
        cable_lengths, motor_steps = self.calc_cable_lengths(newpos)
        
        # Set position for each rail
        for i, rail in enumerate(self.rails):
            stepper_pos = motor_steps[i]
            rail.set_position([stepper_pos])
            
        if homing_axes == "xyz":
            self.need_home = False
            
    def home(self, homing_state):
        """Home the robot - simplified for testing"""
        homing_state.set_axes([0, 1, 2])  # X, Y, Z
        
        # Set to home position
        for i, rail in enumerate(self.rails):
            rail.set_position([0])
            
        self.need_home = False
        
    def check_move(self, move):
        """Check that a move is within the boundaries"""
        end_pos = move.end_pos
        
        # Check if we need to home first
        if self.need_home:
            raise move.move_error("Must home first")
        
        # Check boundaries
        if (end_pos[0] < self.axes_min[0] or end_pos[0] > self.axes_max[0] or
            end_pos[1] < self.axes_min[1] or end_pos[1] > self.axes_max[1] or
            end_pos[2] < self.axes_min[2] or end_pos[2] > self.axes_max[2]):
            raise move.move_error()
            
        # Limit Z movements based on Z speed limits
        if move.axes_d[2]:
            z_ratio = move.move_d / abs(move.axes_d[2])
            move.limit_speed(self.max_z_velocity * z_ratio,
                             self.max_z_accel * z_ratio)
            
    def get_status(self, eventtime):
        return {
            'homed_axes': '' if self.need_home else 'xyz',
            'axis_minimum': self.axes_min,
            'axis_maximum': self.axes_max,
        }

def load_kinematics(toolhead, config):
    return CDPRKinematics(toolhead, config)

# Register CDPR as a valid kinematics type for Klipper
def load_config():
    # Register CDPR with Klipper
    pins.get_printer().add_object('cdpr', load_kinematics)
