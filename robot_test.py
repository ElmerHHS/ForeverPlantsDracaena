#!/usr/bin/env python
import keyboard
from kawapai.robot import KawaBot
import time


# Main workflow
def main():
    robot = KawaBot(host="192.168.0.1", port=23)
    try:
        robot.abort_kill_all()
        robot.motor_power_off()
        robot.reset_error()
        robot.motor_power_on()
        robot.initiate_kawabot()
        robot.connect_to_movement_server()
        robot.connect_to_pose_update_server()

        print('Resetting robot position...')
        move_robot_to_home(robot)
        robot_position = robot.get_current_position(joint=True)
        print('Joint positions: {}'.format(robot_position))
        robot_position = robot.get_current_position()
        print('Carthesian coordinates: {}'.format(robot_position))

        coordinates = [[-100, 400, 150, 0, 90, 0], [100, 400, 150, 0, 90, 0]]

        for coordinate in coordinates:
            print('Press space to move robot to: X: {} Y: {} Z: {}'.format(coordinate[0],
                                                                           coordinate[1],
                                                                           coordinate[2]))
            key2 = keyboard.read_key()
            if key2 == 'space':
                in_position = False
                robot.jmove_cartesian(round(coordinate[0]), round(coordinate[1]),
                                      round(coordinate[2]), round(coordinate[3]),
                                      round(coordinate[4]), round(coordinate[5]))
                while not in_position:
                    robot_position = robot.get_current_position(joint=True)
                    print('Joint positions: {}'.format(robot_position))
                    robot_position = robot.get_current_position()
                    print('Carthesian coordinates: {}'.format(robot_position))
                    if round(robot_position[0]) == round(coordinate[0]) and round(robot_position[1]) == round(coordinate[1]) \
                            and round(robot_position[2]) == round(coordinate[2]):
                        in_position = True

            if key2 == 'q':
                exit(-1)
    finally:
        robot.close_movement_server()
        robot.close_pose_update_server()
        robot.motor_power_off()
        robot.disconnect()


def move_robot_to_home(kawa: KawaBot):
    kawa.jmove_joint_check_pos(0, -50, -105, 5, -38, 0)
    print('In position!')


if __name__ == '__main__':
    main()
