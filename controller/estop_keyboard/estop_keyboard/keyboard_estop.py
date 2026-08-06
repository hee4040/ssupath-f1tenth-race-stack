#!/usr/bin/env python3
"""Keyboard-driven E-STOP publisher node.

Behavior:
 - Press 'e' to toggle E-STOP (latched).
 - Press 'r' to release E-STOP (set false).

If the `keyboard` package is not available, falls back to a simple stdin loop.
"""
import threading
import time

try:
    import keyboard  # type: ignore
    HAS_KEYBOARD = True
except Exception:
    HAS_KEYBOARD = False

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool


class KeyboardEstop(Node):
    def __init__(self):
        super().__init__('keyboard_estop')
        self.pub = self.create_publisher(Bool, '/e_stop', 10)
        self.estop = False
        self.get_logger().info('keyboard_estop started (press e to toggle, r to release)')

        if HAS_KEYBOARD:
            # register keyboard callbacks in a separate thread
            t = threading.Thread(target=self._keyboard_thread, daemon=True)
            t.start()
        else:
            t = threading.Thread(target=self._stdin_thread, daemon=True)
            t.start()

        # publish initial released state
        self._publish()

    def _publish(self):
        msg = Bool()
        msg.data = self.estop
        self.pub.publish(msg)
        self.get_logger().info(f'/e_stop -> {msg.data}')

    def _keyboard_thread(self):
        # using keyboard library callbacks
        keyboard.on_press_key('e', lambda e: self._toggle())
        keyboard.on_press_key('r', lambda e: self._release())
        # keep thread alive
        while rclpy.ok():
            time.sleep(0.1)

    def _stdin_thread(self):
        self.get_logger().warning('keyboard module not available; falling back to stdin. Type e/r then Enter.')
        while rclpy.ok():
            try:
                s = input().strip()
            except EOFError:
                break
            if not s:
                continue
            if 'e' in s:
                self._toggle()
            if 'r' in s:
                self._release()

    def _toggle(self):
        self.estop = not self.estop
        self._publish()

    def _release(self):
        if self.estop:
            self.estop = False
            self._publish()


def main():
    rclpy.init()
    node = KeyboardEstop()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.get_logger().info('keyboard_estop shutting down')
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
