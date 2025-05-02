# Code for handling the kinematics of cable-driven robots (TERS)
#
# Based on original cartesian code:
# Copyright (C) 2016-2021  Kevin O'Connor <kevin@koconnor.net>
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import logging
import math
import stepper
from . import idex_modes

class CableKinematics:
    def __init__(self, toolhead, config):
        self.printer = config.get_printer()
        self.name = config.get_name()
        
        # Cable robot specific parameters
        self.cable_base_length = config.getfloat('cable_base_length', 1.0)
        self.encoder_counts_per_unit = config.getfloat('encoder_counts_per_unit', 5348911.9746)
        
        # Setup cable attachment points (frame anchors)
        # Format: [x, y, z] coordinates for each cable anchor point
        self.anchor_positions = [
            [0.0, 0.0, 0.0],               # Cable 1
            [0.0, 0.438, 0.0],             # Cable 2
            [-0.379, -0.219, 0.0],         # Cable 3
            [0.379, -0.219, 0.0]           # Cable 4
        ]
        
        # Setup end-effector attachment points (relative to end-effector center)
        # Format: [x, y] offsets from center of end-effector for each cable
        self.effector_offsets = [
            [0.0, 0.0],                    # Cable 1
            [0.0, 0.01588],                # Cable 2
            [-0.01375, -0.00794],          # Cable 3
            [0.01375, -0.00794]            # Cable 4
        ]
        
        # Setup steppers
        self.rails = []
        for i in range(4):  # 4 cables/motors
            section = 'stepper_cable%d' % (i+1,)
            rail = stepper.LookupMultiRail(config.getsection(section))
            rail.setup_itersolve('cable_stepper_alloc', ('cable%d' % (i+1,)).encode())
            self.rails.append(rail)
        
        # Register steppers with toolhead
        for s in self.get_steppers():
            s.set_trapq(toolhead.get_trapq())
            toolhead.register_step_generator(s.generate_steps)
        
        # Setup boundary checks
        self.max_velocity, self.max_accel = toolhead.get_max_velocity()
        self.max_z_velocity = config.getfloat('max_z_velocity', self.max_velocity,
                                            above=0., maxval=self.max_velocity)
        self.max_z_accel = config.getfloat('max_z_accel', self.max_accel,
                                         above=0., maxval=self.max_accel)
        
        # Get printer dimensions from config
        self.min_x = config.getfloat('min_x', 0.)
        self.max_x = config.getfloat('max_x', 0.)
        self.min_y = config.getfloat('min_y', 0.)
        self.max_y = config.getfloat('max_y', 0.)
        self.min_z = config.getfloat('min_z', 0.)
        self.max_z = config.getfloat('max_z', 0.)
        
        # Setup axis limits
        self.axes_min = toolhead.Coord(self.min_x, self.min_y, self.min_z, e=0.)
        self.axes_max = toolhead.Coord(self.max_x, self.max_y, self.max_z, e=0.)
        self.limits = [(1.0, -1.0)] * 3  # Homing state for XYZ axes
        
        # Register commands
        gcode = self.printer.lookup_object('gcode')
        gcode.register_command('GET_CABLE_LENGTHS', self.cmd_GET_CABLE_LENGTHS,
                             desc=self.cmd_GET_CABLE_LENGTHS_help)
        
        self.printer.register_event_handler("stepper_enable:motor_off",
                                         self._motor_off)
    
    cmd_GET_CABLE_LENGTHS_help = "Report calculated cable lengths"
    def cmd_GET_CABLE_LENGTHS(self, gcmd):
        # Get current position
        toolhead = self.printer.lookup_object('toolhead')
        pos = toolhead.get_position()
        x, y, z = pos[:3]
        
        # Calculate cable lengths
        lengths, steps = self.calc_cable_lengths(x, y, z)
        
        # Report to user
        msg = "Cable lengths at position (%.3f, %.3f, %.3f):\n" % (x, y, z)
        for i, (length, step) in enumerate(zip(lengths, steps)):
            msg += "Cable %d: %.6f (steps=%.1f)\n" % (i+1, length, step)
        gcmd.respond_info(msg)
    
    def get_steppers(self):
        return [s for rail in self.rails for s in rail.get_steppers()]
    
    def _motor_off(self, print_time):
        # Motor off - mark all axes as no longer homed
        self.limits = [(1.0, -1.0)] * 3
    
    def calc_cable_lengths(self, x, y, z):
        """Calculate cable lengths and steps for a given XYZ position."""
        cable_lengths = []
        cable_steps = []
        
        for i in range(4):
            # Vector from anchor point to effector attachment (with offset)
            lx = x - self.anchor_positions[i][0] + self.effector_offsets[i][0]
            ly = y - self.anchor_positions[i][1] + self.effector_offsets[i][1]
            lz = z - self.anchor_positions[i][2]
            
            # Calculate Euclidean distance (cable length)
            length = math.sqrt(lx*lx + ly*ly + lz*lz)
            cable_lengths.append(length)
            
            # Convert to motor steps
            steps = (length - self.cable_base_length) * self.encoder_counts_per_unit
            cable_steps.append(steps)
        
        return cable_lengths, cable_steps
    
    def calc_position(self, stepper_positions):
        """Return cartesian coordinates for the given cable stepper positions.
        
        This is an approximation as the inverse kinematics for cable robots
        is not trivial to solve directly. In practice, we use an iterative approach
        with the forward kinematics from calc_cable_lengths.
        """
        # This is a placeholder for a proper inverse kinematics solution
        # For now, use the current position as an approximation
        # A real implementation would iterate to find XYZ from cable lengths
        toolhead = self.printer.lookup_object('toolhead')
        return list(toolhead.get_position()[:3])
    
    def set_position(self, newpos, homing_axes):
        """Set the current position in cartesian coordinates."""
        for i, rail in enumerate(self.rails):
            rail.set_position(newpos)
        
        for axis_name in homing_axes:
            axis = "xyz".index(axis_name)
            self.limits[axis] = (self.axes_min[axis], self.axes_max[axis])
    
    def home(self, homing_state):
        """Home the printer according to the homing_state."""
        # Cable robots typically need a more specialized homing routine
        # For now, we'll adapt the cartesian approach
        homing_axes = homing_state.get_axes()
        
        # First, determine if we need to home XYZ together
        home_xyz = all(a in homing_axes for a in "xyz")
        
        if home_xyz:
            # Home all cables simultaneously to a central position
            homepos = [0., 0., self.max_z]  # Using max_z for safety
            forcepos = [0., 0., -1.]  # Force movement down
            
            # Perform homing
            homing_state.home_rails(self.rails, forcepos, homepos)
            
            # Update limits for all axes
            for i in range(3):
                self.limits[i] = (self.axes_min[i], self.axes_max[i])
        else:
            # Individual axis homing - less common for cable robots
            # but implemented for compatibility
            for axis in homing_axes:
                axis_index = "xyz".index(axis)
                
                # Set homing parameters based on axis
                homepos = [None, None, None]
                
                if axis == 'x':
                    homepos[0] = 0.
                elif axis == 'y':
                    homepos[1] = 0.
                elif axis == 'z':
                    homepos[2] = self.max_z
                
                # Determine force direction
                forcepos = list(homepos)
                if axis == 'z':
                    # Z homes in the negative direction
                    forcepos[2] = -1.
                
                # Perform homing
                homing_state.home_rails(self.rails, forcepos, homepos)
                
                # Update limits for this axis
                self.limits[axis_index] = (
                    self.axes_min[axis_index], 
                    self.axes_max[axis_index]
                )
    
    def check_move(self, move):
        """Check that the move is within the printer's limits."""
        # Check axis limits
        end_pos = move.end_pos
        for i in range(3):
            if (move.axes_d[i] and (end_pos[i] < self.limits[i][0] or 
                                  end_pos[i] > self.limits[i][1])):
                if self.limits[i][0] > self.limits[i][1]:
                    raise move.move_error("Must home axis first")
                raise move.move_error()
        
        # Check cable lengths
        x, y, z = end_pos[:3]
        cable_lengths, _ = self.calc_cable_lengths(x, y, z)
        
        # Check that no cable exceeds reasonable limits
        # You may need to adjust these limits based on your machine
        for i, length in enumerate(cable_lengths):
            if length <= 0.01:  # Cable too short
                raise move.move_error("Cable %d would be too short" % (i+1,))
            
            # Check if cable would be too long (adjust max_length as needed)
            max_length = 2.0  # Adjust based on your machine
            if length > max_length:
                raise move.move_error("Cable %d would exceed maximum length" % (i+1,))
        
        # Adjust speed for Z movement
        if not move.axes_d[2]:
            # Normal XY move - use defaults
            return
        
        # Move with Z - update velocity and accel for slower Z axis
        z_ratio = move.move_d / abs(move.axes_d[2])
        move.limit_speed(
            self.max_z_velocity * z_ratio, self.max_z_accel * z_ratio)
    
    def get_status(self, eventtime):
        """Return status for cable printer."""
        axes = [a for a, (l, h) in zip("xyz", self.limits) if l <= h]
        return {
            'homed_axes': "".join(axes),
            'axis_minimum': self.axes_min,
            'axis_maximum': self.axes_max,
        }

def load_kinematics(toolhead, config):
    """Load cable robot kinematics."""
    return CableKinematics(toolhead, config)



​
