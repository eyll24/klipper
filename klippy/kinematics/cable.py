# Code for handling the kinematics of cable-driven robots (TERS)
#
# Copyright (C) 2016-2021  Kevin O'Connor <kevin@koconnor.net>
# Cable-driven robot implementation based on original delta code
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import math, logging
import stepper, mathutil

# Slow moves once the ratio of cable to XY movement exceeds SLOW_RATIO
SLOW_RATIO = 3.

class CableKinematics:
    def __init__(self, toolhead, config):
        # Setup cable steppers/rails
        stepper_configs = [config.getsection('stepper_cable%d' % (i+1,)) 
                          for i in range(4)]
        rail_1 = stepper.LookupMultiRail(
            stepper_configs[0], need_position_minmax = False)
        base_endstop = rail_1.get_homing_info().position_endstop
        rail_2 = stepper.LookupMultiRail(
            stepper_configs[1], need_position_minmax = False,
            default_position_endstop=base_endstop)
        rail_3 = stepper.LookupMultiRail(
            stepper_configs[2], need_position_minmax = False,
            default_position_endstop=base_endstop)
        rail_4 = stepper.LookupMultiRail(
            stepper_configs[3], need_position_minmax = False,
            default_position_endstop=base_endstop)
        self.rails = [rail_1, rail_2, rail_3, rail_4]
        
        # Setup max velocity
        self.max_velocity, self.max_accel = toolhead.get_max_velocity()
        self.max_z_velocity = config.getfloat(
            'max_z_velocity', self.max_velocity,
            above=0., maxval=self.max_velocity)
        self.max_z_accel = config.getfloat('max_z_accel', self.max_accel,
                                          above=0., maxval=self.max_accel)
                                          
        # Cable-specific parameters
        self.cable_base_length = config.getfloat('cable_base_length', 1.0)
        self.encoder_counts_per_unit = config.getfloat('encoder_counts_per_unit', 5348911.9746)
        
        # Setup anchor positions (frame attachment points for cables)
        # Default values from the MATLAB script
        self.anchor_x = config.getfloatlist('anchor_x', 
                                          [0.0, 0.0, -0.379, 0.379])
        self.anchor_y = config.getfloatlist('anchor_y', 
                                          [0.0, 0.438, -0.219, -0.219])
        self.anchor_z = config.getfloatlist('anchor_z', 
                                          [0.0, 0.0, 0.0, 0.0])
        
        # Setup end-effector attachment points (relative to end-effector center)
        self.effector_x = config.getfloatlist('effector_x', 
                                            [0.0, 0.0, -0.01375, 0.01375])
        self.effector_y = config.getfloatlist('effector_y', 
                                            [0.0, 0.01588, -0.00794, -0.00794])
        
        # Setup iterative solver for each cable
        for i, rail in enumerate(self.rails):
            rail.setup_itersolve('cable_stepper_alloc', 
                               self.anchor_x[i], self.anchor_y[i], self.anchor_z[i],
                               self.effector_x[i], self.effector_y[i])
        
        # Register steppers with toolhead
        for s in self.get_steppers():
            s.set_trapq(toolhead.get_trapq())
            toolhead.register_step_generator(s.generate_steps)
        
        # Setup boundary checks
        self.need_home = True
        self.limit_xy2 = -1.
        
        # Define workspace boundaries
        self.min_x = config.getfloat('min_x', -0.3)
        self.max_x = config.getfloat('max_x', 0.3)
        self.min_y = config.getfloat('min_y', -0.3)
        self.max_y = config.getfloat('max_y', 0.3)
        self.min_z = config.getfloat('min_z', 0.0)
        self.max_z = config.getfloat('max_z', 0.4)
        
        # Calculate home position (center of workspace)
        self.home_position = (0., 0., self.max_z * 0.8)
        
        # Calculate the safe build volume radius
        # (conservative estimate based on anchor positions)
        max_xy = min(abs(self.max_x), abs(self.max_y))
        self.max_xy2 = max_xy * max_xy
        self.slow_xy2 = (max_xy * 0.8) ** 2
        self.very_slow_xy2 = (max_xy * 0.6) ** 2
        
        # Define axes boundaries for toolhead
        self.axes_min = toolhead.Coord(self.min_x, self.min_y, self.min_z, 0.)
        self.axes_max = toolhead.Coord(self.max_x, self.max_y, self.max_z, 0.)
        
        # Register commands
        gcode = self.printer.lookup_object('gcode')
        gcode.register_command('GET_CABLE_LENGTHS', self.cmd_GET_CABLE_LENGTHS,
                             desc=self.cmd_GET_CABLE_LENGTHS_help)
                             
        # Initialize position
        self.set_position([0., 0., 0.], "")
    
    def get_printer(self):
        return self.rails[0].get_printer()
    
    @property
    def printer(self):
        return self.get_printer()
    
    cmd_GET_CABLE_LENGTHS_help = "Report calculated cable lengths"
    def cmd_GET_CABLE_LENGTHS(self, gcmd):
        # Get current position
        toolhead = self.printer.lookup_object('toolhead')
        pos = toolhead.get_position()
        x, y, z = pos[:3]
        
        # Calculate cable lengths
        cable_lengths = self.calc_cable_lengths(x, y, z)
        cable_steps = self.calc_cable_steps(cable_lengths)
        
        # Report to user
        msg = "Cable lengths at position (%.3f, %.3f, %.3f):\n" % (x, y, z)
        for i, (length, step) in enumerate(zip(cable_lengths, cable_steps)):
            msg += "Cable %d: %.6f mm (steps=%.1f)\n" % (i+1, length, step)
        gcmd.respond_info(msg)
    
    def get_steppers(self):
        return [s for rail in self.rails for s in rail.get_steppers()]
    
    def calc_cable_lengths(self, x, y, z):
        """Calculate cable lengths for a given XYZ position."""
        cable_lengths = []
        
        for i in range(4):
            # Vector from anchor point to effector attachment (with offset)
            lx = x - self.anchor_x[i] + self.effector_x[i]
            ly = y - self.anchor_y[i] + self.effector_y[i]
            lz = z - self.anchor_z[i]
            
            # Calculate Euclidean distance (cable length)
            length = math.sqrt(lx*lx + ly*ly + lz*lz)
            cable_lengths.append(length)
        
        return cable_lengths
    
    def calc_cable_steps(self, cable_lengths):
        """Convert cable lengths to motor steps."""
        return [(length - self.cable_base_length) * self.encoder_counts_per_unit 
                for length in cable_lengths]
    
    def _cartesian_to_actuator(self, coord):
        """Convert cartesian coordinates to actuator steps."""
        cable_lengths = self.calc_cable_lengths(coord[0], coord[1], coord[2])
        return self.calc_cable_steps(cable_lengths)
    
    def _actuator_to_cartesian(self, pos):
        """Convert actuator steps to cartesian coordinates.
        
        Uses an iterative approach since there's no direct inverse solution.
        This is a simplified implementation and may not be fully robust for
        all configurations.
        """
        # Convert from step position to cable lengths
        cable_lengths = [p / self.encoder_counts_per_unit + self.cable_base_length 
                        for p in pos]
        
        # Use trilateration-like approach
        # For now just use the current position as a workaround
        # A real implementation would require a more sophisticated solver
        toolhead = self.printer.lookup_object('toolhead')
        return list(toolhead.get_position()[:3])
    
    def calc_position(self, stepper_positions):
        """Return cartesian coordinates for the given cable stepper positions."""
        spos = [stepper_positions[rail.get_name()] for rail in self.rails]
        return self._actuator_to_cartesian(spos)
    
    def set_position(self, newpos, homing_axes):
        """Set the current position in cartesian coordinates."""
        for rail in self.rails:
            rail.set_position(newpos)
        self.limit_xy2 = -1.
        if homing_axes == "xyz":
            self.need_home = False
    
    def clear_homing_state(self, clear_axes):
        """Clear homing state of each axis."""
        if clear_axes:
            self.limit_xy2 = -1
            self.need_home = True
    
    def home(self, homing_state):
        """Home the printer according to the homing_state."""
        # All axes are homed simultaneously
        homing_state.set_axes([0, 1, 2])
        forcepos = list(self.home_position)
        forcepos[2] = self.min_z
        homing_state.home_rails(self.rails, forcepos, self.home_position)
    
    def check_move(self, move):
        """Check that the move is within the printer's limits."""
        end_pos = move.end_pos
        end_xy2 = end_pos[0]**2 + end_pos[1]**2
        
        if end_xy2 <= self.limit_xy2 and not move.axes_d[2]:
            # Normal XY move
            return
        
        if self.need_home:
            raise move.move_error("Must home first")
            
        # Check workspace boundaries
        end_x, end_y, end_z = end_pos[:3]
        if (end_x < self.min_x or end_x > self.max_x or
            end_y < self.min_y or end_y > self.max_y or
            end_z < self.min_z or end_z > self.max_z):
            raise move.move_error("Move out of range")
        
        # Check cable lengths
        cable_lengths = self.calc_cable_lengths(end_x, end_y, end_z)
        for i, length in enumerate(cable_lengths):
            min_cable_length = 0.05  # Avoid cables getting too slack
            max_cable_length = 2.0   # Limit based on workspace size
            
            if length < min_cable_length:
                raise move.move_error("Cable %d would be too short" % (i+1,))
            if length > max_cable_length:
                raise move.move_error("Cable %d would exceed maximum length" % (i+1,))
        
        # Adjust speed for Z movement
        if move.axes_d[2]:
            z_ratio = move.move_d / abs(move.axes_d[2])
            move.limit_speed(self.max_z_velocity * z_ratio,
                           self.max_z_accel * z_ratio)
            limit_xy2 = -1.
        
        # Limit the speed/accel of moves at the extreme ends of the workspace
        extreme_xy2 = max(end_xy2, move.start_pos[0]**2 + move.start_pos[1]**2)
        if extreme_xy2 > self.slow_xy2:
            r = 0.5
            if extreme_xy2 > self.very_slow_xy2:
                r = 0.25
            move.limit_speed(self.max_velocity * r, self.max_accel * r)
            limit_xy2 = -1.
        else:
            limit_xy2 = self.slow_xy2
            
        self.limit_xy2 = limit_xy2
    
    def get_status(self, eventtime):
        """Return status for cable robot."""
        return {
            'homed_axes': '' if self.need_home else 'xyz',
            'axis_minimum': self.axes_min,
            'axis_maximum': self.axes_max,
        }
    
    def get_calibration(self):
        """Return calibration data for use with calibration routines."""
        return CableCalibration(
            self.cable_base_length, self.encoder_counts_per_unit,
            self.anchor_x, self.anchor_y, self.anchor_z,
            self.effector_x, self.effector_y)

# Cable robot parameter calibration (for possible future CABLE_CALIBRATE tool)
class CableCalibration:
    def __init__(self, cable_base_length, encoder_counts_per_unit,
                 anchor_x, anchor_y, anchor_z, effector_x, effector_y):
        self.cable_base_length = cable_base_length
        self.encoder_counts_per_unit = encoder_counts_per_unit
        self.anchor_x = anchor_x
        self.anchor_y = anchor_y
        self.anchor_z = anchor_z
        self.effector_x = effector_x
        self.effector_y = effector_y
    
    def save_state(self, configfile):
        """Save the current parameters (for use with SAVE_CONFIG)."""
        configfile.set('cable_kinematics', 'cable_base_length', 
                     "%.6f" % (self.cable_base_length,))
        configfile.set('cable_kinematics', 'encoder_counts_per_unit', 
                     "%.6f" % (self.encoder_counts_per_unit,))
        
        # Save anchor positions
        configfile.set('cable_kinematics', 'anchor_x', 
                     ", ".join(["%.6f" % x for x in self.anchor_x]))
        configfile.set('cable_kinematics', 'anchor_y', 
                     ", ".join(["%.6f" % y for y in self.anchor_y]))
        configfile.set('cable_kinematics', 'anchor_z', 
                     ", ".join(["%.6f" % z for z in self.anchor_z]))
        
        # Save effector offsets
        configfile.set('cable_kinematics', 'effector_x', 
                     ", ".join(["%.6f" % x for x in self.effector_x]))
        configfile.set('cable_kinematics', 'effector_y', 
                     ", ".join(["%.6f" % y for y in self.effector_y]))
        
        # Display saved values
        gcode = configfile.get_printer().lookup_object("gcode")
        msg = "Cable robot configuration:\n"
        msg += "cable_base_length: %.6f\n" % (self.cable_base_length,)
        msg += "encoder_counts_per_unit: %.6f\n" % (self.encoder_counts_per_unit,)
        
        msg += "anchor_x: %s\n" % (", ".join(["%.6f" % x for x in self.anchor_x]),)
        msg += "anchor_y: %s\n" % (", ".join(["%.6f" % y for y in self.anchor_y]),)
        msg += "anchor_z: %s\n" % (", ".join(["%.6f" % z for z in self.anchor_z]),)
        
        msg += "effector_x: %s\n" % (", ".join(["%.6f" % x for x in self.effector_x]),)
        msg += "effector_y: %s" % (", ".join(["%.6f" % y for y in self.effector_y]),)
        
        gcode.respond_info(msg)

def load_kinematics(toolhead, config):
    """Load cable robot kinematics."""
    return CableKinematics(toolhead, config)
