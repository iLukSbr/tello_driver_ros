#!/usr/bin/env python3
"""lesc_terminal_control.py

Interactive terminal client that calls services provided by `lesc_control`.

Commands:
- front [cm]              : step forward by N centimeters (default param)
- back [cm]               : step backward by N centimeters
- left [cm]               : step left by N centimeters
- right [cm]              : step right by N centimeters
- up [cm]                 : step up by N centimeters
- down [cm]               : step down by N centimeters
- rotate [deg]            : rotate around yaw by N degrees (default param)
- moveto x y z [qx qy qz qw] : call move_to_pos (pose in map frame)
- moverel dx dy dz droll dpitch dyaw : call move_relative (Twist: linear + angular)
- setvel lx ly lz ax ay az duration : call set_velocity (publishes for duration locally)
- takeoff
- land
- help
- quit

Notes:
- Compatibility alias: "forward" is still accepted as synonym for "front".
- The command `rotate` rotates around yaw (degrees, default parameter).
- `cw`/`ccw` are accepted as rotate aliases.
"""

import argparse
import math
import time
import rospy
from geometry_msgs.msg import PoseStamped, Twist
from std_srvs.srv import Trigger
from lesc_control.srv import MoveToPosition, MoveRelative, SetVelocity

MOVE_RELATIVE_UNAVAILABLE_MSG = 'move_relative service unavailable'


class LescTerminalControl:
    def __init__(self):
        rospy.init_node('lesc_terminal_control', anonymous=True)

        parser = argparse.ArgumentParser(add_help=False)
        parser.add_argument('--service-namespace', dest='service_namespace', default=None)
        parser.add_argument('--service-wait-timeout', dest='service_wait_timeout', type=float, default=None)
        parser.add_argument('--step-distance-cm', dest='step_distance_cm', type=float, default=None)
        parser.add_argument('--step-angle-deg', dest='step_angle_deg', type=float, default=None)
        args, _ = parser.parse_known_args()

        service_ns_param = rospy.get_param('~service_namespace', '/lesc_control')
        self.service_ns = args.service_namespace if args.service_namespace is not None else service_ns_param
        if not self.service_ns:
            self.service_ns = ''
        if self.service_ns.endswith('/'):
            self.service_ns = self.service_ns[:-1]

        rospy.loginfo('[lesc_terminal_control] usando namespace de serviços: %s', self.service_ns or '/')

        def ns_service(name: str) -> str:
            if self.service_ns:
                return f"{self.service_ns}/{name}"
            return name

        self.srv_move_to = ns_service('move_to_pos')
        self.srv_move_rel = ns_service('move_relative')
        self.srv_set_vel = ns_service('set_velocity')
        self.srv_takeoff = ns_service('takeoff')
        self.srv_land = ns_service('land')

        default_timeout = rospy.get_param('~service_wait_timeout', 3.0)
        self.service_wait_timeout = args.service_wait_timeout if args.service_wait_timeout is not None else default_timeout

        rospy.loginfo('[lesc_terminal_control] waiting services (timeout %.1fs)...', self.service_wait_timeout)

        self.service_specs = [
            ('s_move_to_pos', self.srv_move_to, MoveToPosition),
            ('s_move_relative', self.srv_move_rel, MoveRelative),
            ('s_set_velocity', self.srv_set_vel, SetVelocity),
            ('s_takeoff', self.srv_takeoff, Trigger),
            ('s_land', self.srv_land, Trigger),
        ]

        for attr, _, _ in self.service_specs:
            setattr(self, attr, None)

        for attr, srv_name, srv_type in self.service_specs:
            try:
                rospy.loginfo('  waiting %s', srv_name)
                rospy.wait_for_service(srv_name, timeout=self.service_wait_timeout)
                proxy = rospy.ServiceProxy(srv_name, srv_type)
                setattr(self, attr, proxy)
                rospy.loginfo('  service %s available', srv_name)
            except (rospy.ROSException, rospy.ROSInterruptException):
                rospy.logwarn('  service %s unavailable after %.1fs', srv_name, self.service_wait_timeout)
                setattr(self, attr, None)

        default_step = rospy.get_param('~step_distance_cm', 1.0)
        self.step_distance_cm = args.step_distance_cm if args.step_distance_cm is not None else default_step
        default_angle = rospy.get_param('~step_angle_deg', 15.0)
        self.step_angle_deg = args.step_angle_deg if args.step_angle_deg is not None else default_angle
        rospy.loginfo('[lesc_terminal_control] ready.')

    def _ensure_proxy(self, attr):
        proxy = getattr(self, attr, None)
        if proxy:
            return proxy
        spec = next((s for s in self.service_specs if s[0] == attr), None)
        if not spec:
            rospy.logerr('Unknown service attribute: %s', attr)
            return None
        _, srv_name, srv_type = spec
        try:
            rospy.loginfo('  retry waiting %s', srv_name)
            rospy.wait_for_service(srv_name, timeout=self.service_wait_timeout)
            proxy = rospy.ServiceProxy(srv_name, srv_type)
            setattr(self, attr, proxy)
            rospy.loginfo('  service %s available', srv_name)
            return proxy
        except (rospy.ROSException, rospy.ROSInterruptException):
            rospy.logwarn('  service %s unavailable after %.1fs', srv_name, self.service_wait_timeout)
            return None

    def repl(self):
        print('lesc_control terminal client. type help')
        while not rospy.is_shutdown():
            try:
                line = input('> ').strip()
            except EOFError:
                break
            if not line:
                continue
            parts = line.split()
            cmd = parts[0].lower()
            args = parts[1:]

            try:
                if cmd in ('quit', 'exit'):
                    break
                elif cmd == 'help':
                    print(__doc__)
                elif cmd == 'takeoff':
                    proxy = self._ensure_proxy('s_takeoff')
                    if not proxy:
                        print('takeoff service unavailable')
                        continue
                    resp = proxy()
                    print('takeoff:', resp.success, resp.message)
                elif cmd == 'land':
                    proxy = self._ensure_proxy('s_land')
                    if not proxy:
                        print('land service unavailable')
                        continue
                    resp = proxy()
                    print('land:', resp.success, resp.message)
                elif cmd in ('front', 'forward', 'back', 'left', 'right', 'up', 'down'):
                    proxy = self._ensure_proxy('s_move_relative')
                    if not proxy:
                        print(MOVE_RELATIVE_UNAVAILABLE_MSG)
                        continue
                    dist_cm = float(args[0]) if len(args) >= 1 else self.step_distance_cm
                    meters = dist_cm / 100.0
                    twist = Twist()
                    if cmd in ('front', 'forward'):
                        twist.linear.x = meters
                    elif cmd == 'back':
                        twist.linear.x = -meters
                    elif cmd == 'left':
                        twist.linear.y = meters
                    elif cmd == 'right':
                        twist.linear.y = -meters
                    elif cmd == 'up':
                        twist.linear.z = meters
                    elif cmd == 'down':
                        twist.linear.z = -meters
                    resp = proxy(twist)
                    print('move_relative (step):', resp.success, resp.message)
                elif cmd in ('rotate', 'cw', 'ccw'):
                    proxy = self._ensure_proxy('s_move_relative')
                    if not proxy:
                        print(MOVE_RELATIVE_UNAVAILABLE_MSG)
                        continue
                    angle_deg = float(args[0]) if len(args) >= 1 else self.step_angle_deg
                    if cmd == 'cw':
                        angle_deg = -abs(angle_deg)
                    elif cmd == 'ccw':
                        angle_deg = abs(angle_deg)
                    twist = Twist()
                    twist.angular.z = math.radians(angle_deg)
                    resp = proxy(twist)
                    print('rotate (yaw):', resp.success, resp.message)
                elif cmd == 'stop':
                    proxy = self._ensure_proxy('s_set_velocity')
                    if not proxy:
                        print('set_velocity service unavailable')
                        continue
                    zero = Twist()
                    resp = proxy(zero)
                    print('stop (set_velocity zero):', getattr(resp, 'success', None), getattr(resp, 'message', None))
                elif cmd == 'moveto':
                    proxy = self._ensure_proxy('s_move_to_pos')
                    if not proxy:
                        print('move_to_pos service unavailable')
                        continue
                    if len(args) < 3:
                        print('Usage: moveto x y z [qx qy qz qw]')
                        continue
                    pose = PoseStamped()
                    pose.header.frame_id = 'map'
                    pose.pose.position.x = float(args[0])
                    pose.pose.position.y = float(args[1])
                    pose.pose.position.z = float(args[2])
                    if len(args) >= 7:
                        pose.pose.orientation.x = float(args[3])
                        pose.pose.orientation.y = float(args[4])
                        pose.pose.orientation.z = float(args[5])
                        pose.pose.orientation.w = float(args[6])
                    else:
                        pose.pose.orientation.x = 0.0
                        pose.pose.orientation.y = 0.0
                        pose.pose.orientation.z = 0.0
                        pose.pose.orientation.w = 1.0
                    resp = proxy(pose)
                    print('move_to_pos:', resp.success, resp.message)
                elif cmd == 'moverel':
                    proxy = self._ensure_proxy('s_move_relative')
                    if not proxy:
                        print(MOVE_RELATIVE_UNAVAILABLE_MSG)
                        continue
                    if len(args) < 6:
                        print('Usage: moverel dx dy dz droll dpitch dyaw')
                        continue
                    twist = Twist()
                    twist.linear.x = float(args[0])
                    twist.linear.y = float(args[1])
                    twist.linear.z = float(args[2])
                    twist.angular.x = float(args[3])
                    twist.angular.y = float(args[4])
                    twist.angular.z = float(args[5])
                    resp = proxy(twist)
                    print('move_relative:', resp.success, resp.message)
                elif cmd == 'setvel':
                    proxy = self._ensure_proxy('s_set_velocity')
                    if not proxy:
                        print('set_velocity service unavailable')
                        continue
                    if len(args) < 6:
                        print('Usage: setvel lx ly lz ax ay az [duration]')
                        continue
                    twist = Twist()
                    twist.linear.x = float(args[0])
                    twist.linear.y = float(args[1])
                    twist.linear.z = float(args[2])
                    twist.angular.x = float(args[3])
                    twist.angular.y = float(args[4])
                    twist.angular.z = float(args[5])
                    resp = proxy(twist)
                    print('set_velocity:', resp.success, resp.message)
                    if len(args) >= 7:
                        dur = float(args[6])
                        print('Sleeping for', dur, 's')
                        time.sleep(dur)
                        zero = Twist()
                        proxy_zero = self._ensure_proxy('s_set_velocity')
                        if proxy_zero:
                            proxy_zero(zero)
                else:
                    print('Unknown command', cmd)
            except rospy.ServiceException as e:
                print('Service call failed:', e)


def main():
    ctl = LescTerminalControl()
    ctl.repl()


if __name__ == '__main__':
    main()
