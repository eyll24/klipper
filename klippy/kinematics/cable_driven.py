# Code for handling the kinematics of cable-driven robots
#
# Based on CartKinematics but modified for cable-driven parallel robots
# This file may be distributed under the terms of the GNU GPLv3 license.
import logging
import stepper
import math
from . import idex_modes

class CableDrivenKinematics:
    def __init__(self, toolhead, config):
        self.printer = config.get_printer()
        
        # Cable-driven robot specific parameters
        self.drum_radius = config.getfloat('drum_radius', 31.96)  # Drum radius in mm
        self.steps_per_rotation = config.getfloat('steps_per_rotation', 200)  # Motor steps per rotation
        self.initial_cable_length = config.getfloat('initial_cable_length', 1.0)  # Initial cable length in meters
        
        # Frame attachment points (in meters)
        self.frame_points = [
            [0.0, 0.0, 0.0],            # Origin (not used in cable robot)
            [0.0, 0.438, 0.0],          # Motor 1 position 
            [-0.379, -0.219, 0.0],      # Motor 2 position
            [0.379, -0.219, 0.0]        # Motor 3 position
        ]
        
        # End effector attachment points (local coordinates in meters)
        self.effector_points = [
            [0.0, 0.0, 0.0],            # Origin (not used in cable robot)
            [0.0, 0.01588, 0.0],        # Cable 1 attachment
            [-0.01375, -0.00794, 0.0],  # Cable 2 attachment
            [0.01375, -0.00794, 0.0]    # Cable 3 attachment
        ]
        
        # Calculate conversion factor (steps per meter of cable)
        drum_circumference = 2 * math.pi * (self.drum_radius / 1000)  # Convert mm to m
        self.conv_factor = self.steps_per_rotation / drum_circumference
        
        # Setup axis rails - we still need them for compatibility
        self.dual_carriage_axis = None
        self.dual_carriage_rails = []
        self.rails = [stepper.LookupMultiRail(config.getsection('stepper_' + n))
                      for n in 'xyz']
        
        # Only set up standard Cartesian itersolve for initialization,
        # we'll override the positions with our cable calculations
        for rail, axis in zip(self.rails, 'xyz'):
            rail.setup_itersolve('cartesian_stepper_alloc', axis.encode())
            
        ranges = [r.get_range() for r in self.rails]
        self.axes_min = toolhead.Coord(*[r[0] for r in ranges], e=0.)
        self.axes_max = toolhead.Coord(*[r[1] for r in ranges], e=0.)
        self.dc_module = None
        
        # Handle dual carriage if present (same as original)
        if config.has_section('dual_carriage'):
            dc_config = config.getsection('dual_carriage')
            dc_axis = dc_config.getchoice('axis', ['x', 'y'])
            self.dual_carriage_axis = {'x': 0, 'y': 1}[dc_axis]
            # setup second dual carriage rail
            self.rails.append(stepper.LookupMultiRail(dc_config))
            self.rails[3].setup_itersolve('cartesian_stepper_alloc',
                                          dc_axis.encode())
            dc_rail_0 = idex_modes.DualCarriagesRail(
                    self.rails[self.dual_carriage_axis],
                    axis=self.dual_carriage_axis, active=True)
            dc_rail_1 = idex_modes.DualCarriagesRail(
                    self.rails[3], axis=self.dual_carriage_axis, active=False)
            self.dc_module = idex_modes.DualCarriages(
                    dc_config, dc_rail_0, dc_rail_1,
                    axis=self.dual_carriage_axis)
                    
        for s in self.get_steppers():
            s.set_trapq(toolhead.get_trapq())
            toolhead.register_step_generator(s.generate_steps)
            
        # Setup boundary checks
        max_velocity, max_accel = toolhead.get_max_velocity()
        self.max_z_velocity = config.getfloat('max_z_velocity', max_velocity,
                                              above=0., maxval=max_velocity)
        self.max_z_accel = config.getfloat('max_z_accel', max_accel,
                                           above=0., maxval=max_accel)
        self.limits = [(1.0, -1.0)] * 3
        
    def get_steppers(self):
        return [s for rail in self.rails for s in rail.get_steppers()]
    
    def calc_position(self, stepper_positions):
        # For cable-driven robots, this is a complex inverse kinematics problem
        # For now, we'll return the original calculation for compatibility
        rails = self.rails
        if self.dc_module:
            primary_rail = self.dc_module.get_primary_rail().get_rail()
            rails = (rails[:self.dc_module.axis] +
                     [primary_rail] + rails[self.dc_module.axis+1:])
        return [stepper_positions[rail.get_name()] for rail in rails]
    
    def calculate_cable_lengths(self, pos):
        """Calculate cable lengths (and motor steps) for a given position"""
        x, y, z = pos[:3]
        cable_lengths = []
        motor_steps = []
        
        # Calculate only for cables 1, 2, 3 (indices 1, 2, 3)
        for i in range(1, 4):
            # Cable vector calculation (similar to MATLAB function)
            lx = x - self.frame_points[i][0] + self.effector_points[i][0]
            ly = y - self.frame_points[i][1] + self.effector_points[i][1]
            lz = z - self.frame_points[i][2]
            
            # Cable length calculation
            length = math.sqrt(lx**2 + ly**2 + lz**2)
            cable_lengths.append(length)
            
            # Motor steps calculation (similar to lv in MATLAB)
            steps = (length - self.initial_cable_length) * self.conv_factor
            motor_steps.append(steps)
            
        return cable_lengths, motor_steps
    
    def update_limits(self, i, range):
        l, h = self.limits[i]
        # Only update limits if this axis was already homed,
        # otherwise leave in un-homed state.
        if l <= h:
            self.limits[i] = range
    
    def set_position(self, newpos):
        """Override position setting with cable-driven kinematics"""
        # First calculate cable lengths for the new position
        cable_lengths, motor_steps = self.calculate_cable_lengths(newpos)
        
        # Apply the standard position setting for compatibility
        for i, rail in enumerate(self.rails):
            rail.set_position(newpos)
            
            # For cables 1, 2, 3 (corresponding to rails y, x, z)
            # override the stepper positions based on cable calculations
            if i < 3 and i > 0:  # We only use cables 1, 2, 3 (skip 0 which is not used)
                # Here we would update the stepper position based on cable length
                # This would require extending the rail interface to support direct step setting
                # For now, we'll keep this as a placeholder
                pass
        
      
    
    def clear_homing_state(self, clear_axes):
        pass
                
    def home_axis(self, homing_state, axis, rail):
       pass
        
    def home(self, homing_state):
        pass
                
    def _check_endstops(self, move):
        end_pos = move.end_pos
        for i in (0, 1, 2):
            if (move.axes_d[i]
                and (end_pos[i] < self.limits[i][0]
                     or end_pos[i] > self.limits[i][1])):
                if self.limits[i][0] > self.limits[i][1]:
                    raise move.move_error("Must home axis first")
                raise move.move_error()
                
    def check_move(self, move):
        """Check and transform move for cable-driven kinematics"""
        # First apply the standard boundary checks
        limits = self.limits
        xpos, ypos = move.end_pos[:2]
        if (xpos < limits[0][0] or xpos > limits[0][1]
            or ypos < limits[1][0] or ypos > limits[1][1]):
            self._check_endstops(move)
            
        # Calculate cable lengths for the end position
        cable_lengths, motor_steps = self.calculate_cable_lengths(move.end_pos)
        
        # Here we should transform the movement into cable-space
        # This would require modifying the move planner, which is beyond
        # a simple file modification
        
        # Apply Z-axis velocity limiting as in the original code
        if not move.axes_d[2]:
            # Normal XY move - use defaults
            return
        # Move with Z - update velocity and accel for slower Z axis
        self._check_endstops(move)
        z_ratio = move.move_d / abs(move.axes_d[2])
        move.limit_speed(
            self.max_z_velocity * z_ratio, self.max_z_accel * z_ratio)
            
    def get_status(self, eventtime):
        axes = [a for a, (l, h) in zip("xyz", self.limits) if l <= h]
        return {
            'homed_axes': "".join(axes),
            'axis_minimum': self.axes_min,
            'axis_maximum': self.axes_max,
        }

def load_kinematics(toolhead, config):
    """Load cable-driven kinematics module"""
    return CableDrivenKinematics(toolhead, config)

