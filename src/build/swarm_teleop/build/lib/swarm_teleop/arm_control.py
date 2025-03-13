#!/usr/bin/env python3
import sys
import rclpy
import std_msgs.msg
import termios
import tty

msg = """
Press SPACE to arm/disarm the drone.
"""

def getKey(settings):
    tty.setraw(sys.stdin.fileno())
    key = sys.stdin.read(1)
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

def saveTerminalSettings():
    return termios.tcgetattr(sys.stdin)

def restoreTerminalSettings(old_settings):
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)

def main():
    settings = saveTerminalSettings()
    rclpy.init()
    node = rclpy.create_node('drone_arm_control')

    arm_toggle = False
    arm_pub = node.create_publisher(std_msgs.msg.Bool, '/arm_message', 10)

    try:
        print(msg)
        while True:
            key = getKey(settings)

            if key == ' ':  # Spacebar for arming/disarming
                arm_toggle = not arm_toggle
                arm_msg = std_msgs.msg.Bool()
                arm_msg.data = arm_toggle
                arm_pub.publish(arm_msg)
                print(f"Arm status: {arm_toggle}")

            elif key == '\x03':  # Ctrl+C to exit
                break

    except Exception as e:
        print(e)

    finally:
        restoreTerminalSettings(settings)

if __name__ == '__main__':
    main()
