#!/usr/bin/env python3
"""joy_tello_test.py

Script de teste para controlar o Tello usando um joystick via ROS.

Funcionalidades principais:
- Mapeamentos pré-configurados (DualSense/PS5 por padrão) com possibilidade de override via parâmetros.
- Publica TwistStamped em /tello/cmd_vel_stamped e Twist em /tello/cmd_vel.
- Comandos discretos (steps) usando o D-pad: cada toque desloca ~1 cm por padrão.
- Botões dedicados para decolagem, pouso e emergência.
- Suporte a modos "lento"/"rápido" alterando o ganho do stick.

Parâmetros relevantes:
- ~controller_model (str): 'ps5' (default) ou 'manual'.
- ~step_distance_cm (float): distância dos comandos discretos (default 1.0).
- ~step_time (float): duração em segundos do movimento discreto (default 0.2).
- ~publish_rate (float): frequência de publicação das velocidades contínuas (default 20.0 Hz).
- ~max_linear_speed (float): velocidade linear máxima (m/s) para eixo X/Y/Z (default 0.5).
- ~max_vertical_speed (float): velocidade vertical máxima (m/s) (default 0.5).
- ~max_yaw_speed (float): velocidade angular máxima (rad/s) (default 1.5).
- ~deadzone (float): zona morta aplicada aos eixos (default 0.05).
- ~slow_factor / ~fast_factor (float): multiplicadores aplicados quando botões de modo são pressionados.
- Quando controller_model='manual', parâmetros individuais de mapeamento podem ser passados:
  * ~axis_pitch, ~axis_roll, ~axis_yaw, ~axis_throttle
  * ~button_takeoff, ~button_land, ~button_emergency, ~button_slow, ~button_fast
  * ~dpad_horizontal_axis, ~dpad_vertical_axis
"""

from dataclasses import dataclass
import math
import time
from typing import Optional

import rospy
from geometry_msgs.msg import Twist, TwistStamped
from sensor_msgs.msg import Joy
from std_msgs.msg import Empty


@dataclass
class ControllerMapping:
    axis_pitch: int
    axis_roll: int
    axis_yaw: int
    axis_throttle: int
    button_takeoff: int
    button_land: int
    button_emergency: int
    button_slow: int
    button_fast: int
    dpad_horizontal_axis: int
    dpad_vertical_axis: int


DEFAULT_PRESETS = {
    "ps5": ControllerMapping(
        axis_pitch=1,            # eixo vertical do stick esquerdo (para frente/trás)
        axis_roll=0,             # eixo horizontal do stick esquerdo (para direita/esquerda)
        axis_yaw=3,              # eixo horizontal do stick direito (yaw)
        axis_throttle=4,         # eixo vertical do stick direito (cima/baixo)
        button_takeoff=3,        # triângulo
        button_land=0,           # quadrado
        button_emergency=1,      # círculo
        button_slow=4,           # L1
        button_fast=5,           # R1
        dpad_horizontal_axis=6,  # esquerda/direita
        dpad_vertical_axis=7,    # cima/baixo
    )
}


class JoyTelloTest:
    def __init__(self):
        rospy.init_node('joy_tello_test', anonymous=True)

        self.publish_rate = rospy.get_param('~publish_rate', 20.0)
        self.step_distance_cm = rospy.get_param('~step_distance_cm', 1.0)
        self.step_time = rospy.get_param('~step_time', 0.2)
        self.max_linear_speed = rospy.get_param('~max_linear_speed', 0.5)
        self.max_vertical_speed = rospy.get_param('~max_vertical_speed', 0.5)
        self.max_yaw_speed = rospy.get_param('~max_yaw_speed', 1.5)
        self.deadzone = rospy.get_param('~deadzone', 0.05)
        self.slow_factor = rospy.get_param('~slow_factor', 0.4)
        self.fast_factor = rospy.get_param('~fast_factor', 1.5)

        model = rospy.get_param('~controller_model', 'ps5').lower()
        if model == 'manual':
            self.mapping = ControllerMapping(
                axis_pitch=rospy.get_param('~axis_pitch'),
                axis_roll=rospy.get_param('~axis_roll'),
                axis_yaw=rospy.get_param('~axis_yaw'),
                axis_throttle=rospy.get_param('~axis_throttle'),
                button_takeoff=rospy.get_param('~button_takeoff'),
                button_land=rospy.get_param('~button_land'),
                button_emergency=rospy.get_param('~button_emergency'),
                button_slow=rospy.get_param('~button_slow'),
                button_fast=rospy.get_param('~button_fast'),
                dpad_horizontal_axis=rospy.get_param('~dpad_horizontal_axis'),
                dpad_vertical_axis=rospy.get_param('~dpad_vertical_axis'),
            )
        else:
            if model not in DEFAULT_PRESETS:
                rospy.logwarn('controller_model %s não encontrado, usando ps5', model)
                model = 'ps5'
            self.mapping = DEFAULT_PRESETS[model]
        rospy.loginfo('joy_tello_test usando preset: %s', model)

        self.tello_cmd_vel_stamped_topic = rospy.get_param('~tello_cmd_vel_stamped_topic', '/tello/cmd_vel_stamped')
        self.tello_cmd_vel_topic = rospy.get_param('~tello_cmd_vel_topic', '/tello/cmd_vel')
        self.tello_takeoff_topic = rospy.get_param('~tello_takeoff_topic', '/tello/takeoff')
        self.tello_land_topic = rospy.get_param('~tello_land_topic', '/tello/land')
        self.tello_emergency_topic = rospy.get_param('~tello_emergency_topic', '/tello/emergency')

        self.pub_vel_stamped = rospy.Publisher(self.tello_cmd_vel_stamped_topic, TwistStamped, queue_size=1)
        self.pub_vel = rospy.Publisher(self.tello_cmd_vel_topic, Twist, queue_size=1)
        self.pub_takeoff = rospy.Publisher(self.tello_takeoff_topic, Empty, queue_size=1)
        self.pub_land = rospy.Publisher(self.tello_land_topic, Empty, queue_size=1)
        self.pub_emergency = rospy.Publisher(self.tello_emergency_topic, Empty, queue_size=1)

        self.current_twist = Twist()
        self.prev_buttons: Optional[list[int]] = None
        self.prev_axes: Optional[list[float]] = None
        self.step_twist: Optional[Twist] = None
        self.step_end_time: float = 0.0

        self.joy_sub = rospy.Subscriber('joy', Joy, self.joy_callback, queue_size=1)
        self.timer = rospy.Timer(rospy.Duration(1.0 / max(self.publish_rate, 1e-3)), self.timer_cb)

        rospy.loginfo('joy_tello_test pronto. Aguardando mensagens de joystick...')

    def joy_callback(self, msg: Joy):
        if self.prev_buttons is None:
            self.prev_buttons = list(msg.buttons)
        if self.prev_axes is None:
            self.prev_axes = list(msg.axes)

        slow_pressed = self._button_pressed(msg.buttons, self.mapping.button_slow)
        fast_pressed = self._button_pressed(msg.buttons, self.mapping.button_fast)
        speed_factor = 1.0
        if slow_pressed and not fast_pressed:
            speed_factor = self.slow_factor
        elif fast_pressed and not slow_pressed:
            speed_factor = self.fast_factor

        pitch = self._axis_value(msg.axes, self.mapping.axis_pitch)
        roll = self._axis_value(msg.axes, self.mapping.axis_roll)
        yaw = self._axis_value(msg.axes, self.mapping.axis_yaw)
        throttle = self._axis_value(msg.axes, self.mapping.axis_throttle)

        pitch = self._apply_deadzone(pitch)
        roll = self._apply_deadzone(roll)
        yaw = self._apply_deadzone(yaw)
        throttle = self._apply_deadzone(throttle)

        self.current_twist.linear.x = -pitch * self.max_linear_speed * speed_factor
        self.current_twist.linear.y = roll * self.max_linear_speed * speed_factor
        self.current_twist.linear.z = throttle * self.max_vertical_speed * speed_factor
        self.current_twist.angular.x = 0.0
        self.current_twist.angular.y = 0.0
        self.current_twist.angular.z = yaw * self.max_yaw_speed * speed_factor

        self._handle_button_event(msg.buttons, self.mapping.button_takeoff, self.pub_takeoff, 'Takeoff')
        self._handle_button_event(msg.buttons, self.mapping.button_land, self.pub_land, 'Land')
        self._handle_button_event(msg.buttons, self.mapping.button_emergency, self.pub_emergency, 'Emergency')

        self._handle_dpad(msg.axes)

        self.prev_buttons = list(msg.buttons)
        self.prev_axes = list(msg.axes)

    def timer_cb(self, event):
        now = rospy.Time.now()
        twist_msg = TwistStamped()
        twist_msg.header.stamp = now

        active_twist = self.current_twist
        if self.step_twist is not None and time.time() < self.step_end_time:
            active_twist = self.step_twist
        else:
            self.step_twist = None

        twist_msg.twist = active_twist
        self.pub_vel_stamped.publish(twist_msg)
        self.pub_vel.publish(twist_msg.twist)

    def _handle_dpad(self, axes):
        horiz = self._axis_value(axes, self.mapping.dpad_horizontal_axis)
        vert = self._axis_value(axes, self.mapping.dpad_vertical_axis)
        prev_horiz = self._axis_value(self.prev_axes, self.mapping.dpad_horizontal_axis) if self.prev_axes else 0.0
        prev_vert = self._axis_value(self.prev_axes, self.mapping.dpad_vertical_axis) if self.prev_axes else 0.0

        def is_pressed(value, prev):
            return value > 0.5 and prev <= 0.5

        def is_pressed_neg(value, prev):
            return value < -0.5 and prev >= -0.5

        if is_pressed(vert, prev_vert):
            self._trigger_step(1.0, 0.0, 0.0)
        elif is_pressed_neg(vert, prev_vert):
            self._trigger_step(-1.0, 0.0, 0.0)

        if is_pressed(horiz, prev_horiz):
            self._trigger_step(0.0, -1.0, 0.0)
        elif is_pressed_neg(horiz, prev_horiz):
            self._trigger_step(0.0, 1.0, 0.0)

    def _trigger_step(self, dir_x: float, dir_y: float, dir_z: float):
        dist_m = self.step_distance_cm / 100.0
        duration = max(self.step_time, 1e-3)
        speed = dist_m / duration
        twist = Twist()
        twist.linear.x = dir_x * speed
        twist.linear.y = dir_y * speed
        twist.linear.z = dir_z * speed
        twist.angular.z = 0.0
        self.step_twist = twist
        self.step_end_time = time.time() + duration
        rospy.loginfo_throttle(1.0, 'Step triggered vx=%.3f vy=%.3f vz=%.3f', twist.linear.x, twist.linear.y, twist.linear.z)

    def _handle_button_event(self, buttons, index, publisher, label: str):
        if index < 0:
            return
        current = self._button_pressed(buttons, index)
        previous = self._button_pressed(self.prev_buttons, index)
        if current and not previous:
            rospy.loginfo('%s button pressed', label)
            publisher.publish(Empty())

    @staticmethod
    def _button_pressed(buttons, index):
        if buttons is None or index is None or index < 0:
            return False
        if index >= len(buttons):
            return False
        return bool(buttons[index])

    @staticmethod
    def _axis_value(axes, index):
        if axes is None or index is None or index < 0:
            return 0.0
        if index >= len(axes):
            return 0.0
        return axes[index]

    def _apply_deadzone(self, value: float) -> float:
        if abs(value) < self.deadzone:
            return 0.0
        return value


def main():
    JoyTelloTest()
    rospy.spin()


if __name__ == '__main__':
    main()
