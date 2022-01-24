#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Copyright (c) 2013 PAL Robotics SL.
# Released under the BSD License.
#
# Authors:
#   * Siegfried-A. Gevatter
# Modified by:
#   * Reijgwart, Victor

import curses
import math

import rospy
from geometry_msgs.msg import Twist


class Velocity():
    def __init__(self, min_velocity, max_velocity, num_steps):
        assert min_velocity > 0 and max_velocity > 0 and num_steps > 0
        self._min = min_velocity
        self._max = max_velocity
        self._num_steps = num_steps
        if self._num_steps > 1:
            self._step_incr = (max_velocity -
                               min_velocity) / (self._num_steps - 1)
        else:
            # If num_steps is one, we always use the minimum velocity.
            self._step_incr = 0

    def __call__(self, value, step):
        """
        Takes a value in the range [0, 1] and the step and returns the
        velocity (usually m/s or rad/s).
        """
        if step == 0:
            return 0

        assert 0 < step <= self._num_steps
        max_value = self._min + self._step_incr * (step - 1)
        return value * max_value


class TextWindow():
    _screen = None
    _window = None
    _num_lines = None

    def __init__(self, stdscr, lines=10):
        self._screen = stdscr
        self._screen.nodelay(True)
        curses.curs_set(0)

        self._num_lines = lines

    def read_key(self):
        keycode = self._screen.getch()
        return keycode if keycode != -1 else None

    def clear(self):
        self._screen.clear()

    def write_line(self, lineno, message):
        if lineno < 0 or lineno >= self._num_lines:
            raise ValueError('lineno out of bounds')
        height, width = self._screen.getmaxyx()
        y = int((height / self._num_lines) * lineno)
        x = 10
        for text in message.split('\n'):
            text = text.ljust(width)
            self._screen.addstr(y, x, text)
            y += 1

    def refresh(self):
        self._screen.refresh()

    @staticmethod
    def beep():
        curses.flash()


class KeyTeleop():
    # pylint: disable=too-many-instance-attributes
    _interface = None

    _linear = None
    _angular = None

    def __init__(self, interface):
        self._interface = interface
        self._pub_cmd = rospy.Publisher('key_vel', Twist)

        self._hz = rospy.get_param('~hz', 10)

        self._num_steps = rospy.get_param('~turbo/steps', 4)

        forward_min = rospy.get_param('~turbo/linear_forward_min', 0.5)
        forward_max = rospy.get_param('~turbo/linear_forward_max', 1.0)
        self._forward = Velocity(forward_min, forward_max, self._num_steps)

        backward_min = rospy.get_param('~turbo/linear_backward_min', 0.25)
        backward_max = rospy.get_param('~turbo/linear_backward_max', 0.5)
        self._backward = Velocity(backward_min, backward_max, self._num_steps)

        angular_min = rospy.get_param('~turbo/angular_min', 0.7)
        angular_max = rospy.get_param('~turbo/angular_max', 1.2)
        self._rotation = Velocity(angular_min, angular_max, self._num_steps)

    def run(self):
        self._linear = 0
        self._angular = 0

        rate = rospy.Rate(self._hz)
        while True:
            keycode = self._interface.read_key()
            if keycode:
                if self._key_pressed(keycode):
                    self._publish()
            else:
                self._publish()
                rate.sleep()

    def _get_twist(self, linear, angular):
        twist = Twist()
        if linear >= 0:
            twist.linear.x = self._forward(1.0, linear)
        else:
            twist.linear.x = self._backward(-1.0, -linear)
        twist.angular.z = self._rotation(math.copysign(1, angular),
                                         abs(angular))
        return twist

    def _key_pressed(self, keycode):
        movement_bindings = {
            curses.KEY_UP: (1, 0),
            curses.KEY_DOWN: (-1, 0),
            curses.KEY_LEFT: (0, 1),
            curses.KEY_RIGHT: (0, -1),
        }
        speed_bindings = {
            ord(' '): (0, 0),
        }
        if keycode in movement_bindings:
            acc = movement_bindings[keycode]
            ok = False
            if acc[0]:
                linear = self._linear + acc[0]
                if abs(linear) <= self._num_steps:
                    self._linear = linear
                    ok = True
            if acc[1]:
                angular = self._angular + acc[1]
                if abs(angular) <= self._num_steps:
                    self._angular = angular
                    ok = True
            if not ok:
                self._interface.beep()
        elif keycode in speed_bindings:
            acc = speed_bindings[keycode]
            # Note: bounds aren't enforced here!
            if acc[0] is not None:
                self._linear = acc[0]
            if acc[1] is not None:
                self._angular = acc[1]

        elif keycode == ord('q'):
            rospy.signal_shutdown('Bye')
        else:
            return False

        return True

    def _publish(self):
        self._interface.clear()
        self._interface.write_line(
            2, f'Linear: {self._linear}, Angular: {self._angular}')
        self._interface.write_line(
            5, 'Use arrow keys to move, space to stop, q to exit.')
        self._interface.refresh()

        twist = self._get_twist(self._linear, self._angular)
        self._pub_cmd.publish(twist)


class SimpleKeyTeleop():
    # pylint: disable=too-many-instance-attributes
    _running = False

    def __init__(self, interface):
        self._interface = interface
        self._pub_cmd = rospy.Publisher('key_vel', Twist)

        self._hz = rospy.get_param('~hz', 10)

        self._forward_rate = rospy.get_param('~forward_rate', 0.8)
        self._backward_rate = rospy.get_param('~backward_rate', 0.5)
        self._rotation_rate = rospy.get_param('~rotation_rate', 1.0)
        self._last_pressed = {}
        self._angular = 0
        self._linear = 0

    movement_bindings = {
        curses.KEY_UP: (1, 0),
        curses.KEY_DOWN: (-1, 0),
        curses.KEY_LEFT: (0, 1),
        curses.KEY_RIGHT: (0, -1),
    }

    def run(self):
        rate = rospy.Rate(self._hz)
        self._running = True
        while self._running:
            while True:
                keycode = self._interface.read_key()
                if keycode is None:
                    break
                self._key_pressed(keycode)
            self._set_velocity()
            self._publish()
            rate.sleep()

    @staticmethod
    def _get_twist(linear, angular):
        twist = Twist()
        twist.linear.x = linear
        twist.angular.z = angular
        return twist

    def _set_velocity(self):
        now = rospy.get_time()
        keys = []
        for key, value in self._last_pressed.items():
            if now - value < 0.4:
                keys.append(key)
        linear = 0.0
        angular = 0.0
        for k in keys:
            l, key = self.movement_bindings[k]
            linear += l
            angular += key
        if linear > 0:
            linear = linear * self._forward_rate
        else:
            linear = linear * self._backward_rate
        angular = angular * self._rotation_rate
        self._angular = angular
        self._linear = linear

    def _key_pressed(self, keycode):
        if keycode == ord('q'):
            self._running = False
            rospy.signal_shutdown('Bye')
        elif keycode in self.movement_bindings:
            self._last_pressed[keycode] = rospy.get_time()

    def _publish(self):
        self._interface.clear()
        self._interface.write_line(
            2, f'Linear: {self._linear}, Angular: {self._angular}')
        self._interface.write_line(5, 'Use arrow keys to move, q to exit.')
        self._interface.refresh()

        twist = self._get_twist(self._linear, self._angular)
        self._pub_cmd.publish(twist)


def main(stdscr):
    rospy.init_node('key_teleop')
    app = KeyTeleop(TextWindow(stdscr))
    app.run()


if __name__ == '__main__':
    try:
        curses.wrapper(main)
    except rospy.ROSInterruptException:
        pass
