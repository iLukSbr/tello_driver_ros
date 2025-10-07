#!/usr/bin/env python3
"""tello_terminal_control.py

Controle interativo do Tello via terminal (sem joystick).

Comandos:
- takeoff                 : decola (publica Empty em /tello/takeoff)
- land                    : pousa (publica Empty em /tello/land)
- front [cm]              : move para frente X centímetros (default 1)
- back [cm]               : move para trás X centímetros
- left [cm]               : move para esquerda X centímetros
- right [cm]              : move para direita X centímetros
- up [cm]                 : sobe X centímetros
- down [cm]               : desce X centímetros
- rotate [graus]          : gira em yaw o ângulo informado (default parâmetro)
- vel lx ly lz az dur     : publica velocidade linear/ang por dur segundos
- stop                    : publica zero velocity
- help                    : mostra ajuda
- quit / exit             : sai

Notas:
- Publica TwistStamped em /tello/cmd_vel_stamped e /tello/cmd_vel
- Publica Empty em /tello/takeoff e /tello/land
- Alias de compatibilidade: o comando "forward" continua aceito como sinônimo de "front".
  "cw"/"ccw" são aceitos como sinônimos de rotate positivo/negativo.
"""

import argparse
import math
import time
import rospy
from geometry_msgs.msg import TwistStamped
from std_msgs.msg import Empty, Header


class TelloTerminalControl:
    def __init__(self, step_distance_cm=1.0, step_time=0.2, pub_rate=20.0, step_angle_deg=15.0):
        rospy.init_node("tello_terminal_control", anonymous=True)

        self.tello_cmd_vel_stamped_topic = rospy.get_param(
            "~tello_cmd_vel_stamped_topic", "/tello/cmd_vel_stamped"
        )
        self.tello_cmd_vel_topic = rospy.get_param("~tello_cmd_vel_topic", "/tello/cmd_vel")
        self.tello_takeoff_topic = rospy.get_param("~tello_takeoff_topic", "/tello/takeoff")
        self.tello_land_topic = rospy.get_param("~tello_land_topic", "/tello/land")

        self.pub_vel_stamped = rospy.Publisher(self.tello_cmd_vel_stamped_topic, TwistStamped, queue_size=1)
        self.pub_vel = rospy.Publisher(self.tello_cmd_vel_topic, TwistStamped, queue_size=1)
        self.pub_takeoff = rospy.Publisher(self.tello_takeoff_topic, Empty, queue_size=1)
        self.pub_land = rospy.Publisher(self.tello_land_topic, Empty, queue_size=1)

        self.step_distance_cm = step_distance_cm
        self.step_time = step_time
        self.step_angle_deg = step_angle_deg
        self.pub_rate = pub_rate

        rospy.sleep(0.2)

    def publish_empty(self, pub):
        pub.publish(Empty())

    def publish_twist_for(self, linx=0.0, liny=0.0, linz=0.0, angz=0.0, duration=0.2):
        rate = rospy.Rate(self.pub_rate)
        msg = TwistStamped()
        t_end = time.time() + duration
        while not rospy.is_shutdown() and time.time() < t_end:
            msg.header = Header()
            msg.header.stamp = rospy.Time.now()
            msg.twist.linear.x = linx
            msg.twist.linear.y = liny
            msg.twist.linear.z = linz
            msg.twist.angular.z = angz
            self.pub_vel_stamped.publish(msg)
            try:
                rate.sleep()
            except rospy.ROSInterruptException:
                break

    def step_move(self, dir_vec, distance_cm=None, time_s=None):
        if distance_cm is None:
            distance_cm = self.step_distance_cm
        if time_s is None:
            time_s = self.step_time
        speed = (distance_cm / 100.0) / max(time_s, 1e-6)
        linx = dir_vec[0] * speed
        liny = dir_vec[1] * speed
        linz = dir_vec[2] * speed
        self.publish_twist_for(linx, liny, linz, 0.0, time_s)

    def repl(self):
        print("Tello terminal control — digite 'help' para ver comandos")
        try:
            while not rospy.is_shutdown():
                try:
                    cmd = input('> ').strip()
                except EOFError:
                    break
                if not cmd:
                    continue
                parts = cmd.split()
                c = parts[0].lower()
                args = parts[1:]

                if c in ('quit', 'exit'):
                    break
                elif c == 'help':
                    print(__doc__)
                elif c == 'takeoff':
                    confirm = input('Confirm takeoff? (yes/no): ').strip().lower()
                    if confirm in ('yes', 'y'):
                        print('Publishing takeoff')
                        self.publish_empty(self.pub_takeoff)
                    else:
                        print('Takeoff cancelled')
                elif c == 'land':
                    confirm = input('Confirm land? (yes/no): ').strip().lower()
                    if confirm in ('yes', 'y'):
                        print('Publishing land')
                        self.publish_empty(self.pub_land)
                    else:
                        print('Land cancelled')
                elif c in ('front', 'forward', 'back', 'left', 'right', 'up', 'down'):
                    distance = float(args[0]) if args else self.step_distance_cm
                    if c in ('front', 'forward'):
                        self.step_move([1.0, 0.0, 0.0], distance)
                    elif c == 'back':
                        self.step_move([-1.0, 0.0, 0.0], distance)
                    elif c == 'left':
                        self.step_move([0.0, 1.0, 0.0], distance)
                    elif c == 'right':
                        self.step_move([0.0, -1.0, 0.0], distance)
                    elif c == 'up':
                        self.step_move([0.0, 0.0, 1.0], distance)
                    elif c == 'down':
                        self.step_move([0.0, 0.0, -1.0], distance)
                elif c == 'rotate' or c in ('cw', 'ccw'):
                    angle_deg = float(args[0]) if args else self.step_angle_deg
                    if c == 'ccw':
                        angle_deg = abs(angle_deg)
                    elif c == 'cw':
                        angle_deg = -abs(angle_deg)
                    self.publish_twist_for(0.0, 0.0, 0.0, math.radians(angle_deg), self.step_time)
                elif c == 'vel':
                    if len(args) < 5:
                        print('Usage: vel lx ly lz angz duration')
                        continue
                    lx = float(args[0])
                    ly = float(args[1])
                    lz = float(args[2])
                    az = float(args[3])
                    dur = float(args[4])
                    self.publish_twist_for(lx, ly, lz, az, dur)
                elif c == 'stop':
                    self.publish_twist_for(0.0, 0.0, 0.0, 0.0, 0.1)
                else:
                    print('Comando não reconhecido. Digite help para lista.')
        except KeyboardInterrupt:
            pass


def main():
    parser = argparse.ArgumentParser(description='Tello terminal control')
    parser.add_argument('--step-distance-cm', type=float, default=1.0, help='distance in cm for step commands')
    parser.add_argument('--step-time', type=float, default=0.2, help='time in seconds for step commands')
    parser.add_argument('--rate', type=float, default=20.0, help='publish rate (Hz)')
    parser.add_argument('--step-angle-deg', type=float, default=15.0, help='yaw degrees per rotate command')
    ns = parser.parse_args()

    ctrl = TelloTerminalControl(step_distance_cm=ns.step_distance_cm,
                                step_time=ns.step_time,
                                pub_rate=ns.rate,
                                step_angle_deg=ns.step_angle_deg)
    ctrl.repl()


if __name__ == '__main__':
    main()
