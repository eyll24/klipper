# Code for handling the kinematics of cable winch robots
#
# Copyright (C) 2018-2021  Kevin O'Connor <kevin@koconnor.net>
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import stepper, mathutil

class WinchKinematics:
    def __init__(self, toolhead, config):
        # Setup steppers at each anchor
        self.steppers = []
        self.anchors = []
        self.offsets = []
        for i in range(26):
            name = 'stepper_' + chr(ord('a') + i)
            if i >= 3 and not config.has_section(name):
                break
            stepper_config = config.getsection(name)
            s = stepper.PrinterStepper(stepper_config)
            self.steppers.append(s)
            a = tuple([stepper_config.getfloat('anchor_' + n) for n in 'xyz'])
            self.anchors.append(a)
            s.setup_itersolve('winch_stepper_alloc', *a)
            s.set_trapq(toolhead.get_trapq())
            toolhead.register_step_generator(s.generate_steps)
            # Get offset for this specific stepper
            qx = stepper_config.getfloat("offset_x", default=0.0)
            qy = stepper_config.getfloat("offset_y", default=0.0)
            qz = stepper_config.getfloat("offset_z", default=0.0)
            self.offsets.append((qx, qy, qz))
        # Setup boundary checks
        acoords = list(zip(*self.anchors))
        self.axes_min = toolhead.Coord(*[min(a) for a in acoords], e=0.)
        self.axes_max = toolhead.Coord(*[max(a) for a in acoords], e=0.)
        self.set_position([0., 0., 0.], "")
        
    def get_steppers(self):
        return list(self.steppers)
    
    def calc_position(self, stepper_positions):
        # Get cable lengths (not squared yet)
        pos = [stepper_positions[rail.get_name()] for rail in self.steppers[:3]]
        # Apply anchor position offsets: A_i + q_i
        effective_anchors = [
            (a[0] + q[0], a[1] + q[1], a[2] + q[2])
            for a, q in zip(self.anchors[:3], self.offsets[:3]) ]
        # Forward kinematics: solve for P such that ||P - (A_i + q_i)|| = L_i
        return mathutil.trilateration(effective_anchors, [p * p for p in pos])
    
    def set_position(self, newpos, homing_axes):
        for s in self.steppers:
            s.set_position(newpos)
    
    def clear_homing_state(self, clear_axes):
        # XXX - homing not implemented
        pass
    
    def home(self, homing_state):
        # XXX - homing not implemented
        homing_state.set_axes([0, 1, 2])
        homing_state.set_homed_position([0., 0., 0.])
    
    def check_move(self, move):
        # XXX - boundary checks and speed limits not implemented
        pass
    
    def get_status(self, eventtime):
        # XXX - homed_checks and rail limits not implemented
        return {
            'homed_axes': 'xyz',
            'axis_minimum': self.axes_min,
            'axis_maximum': self.axes_max,
        }

def load_kinematics(toolhead, config):
    return WinchKinematics(toolhead, config)
