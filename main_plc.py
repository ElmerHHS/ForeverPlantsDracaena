#!/usr/bin/env python
import time
import numpy as np
import cv2 as cv
from DracaenaVision import DracaenaVision
from kawa_calibration import translate_coordinates_45cm, translate_coordinates_35cm
from DracaenaPLC import DracaenaPLC, DracaenaCodes
from DracaenaCamera import DracaenaCamera
from DracaenaRobot import DracaenaRobot


# Detection image window
window_name = 'Detections'
full_screen = True
# Set to false if no robot is connected to the PC
use_robot = True
# Set to false to only take colour + depth picture and not process them
use_vision = True


# Main workflow
def main():
    # Initialise variables
    robot = None
    # PLC socket communication
    plc = DracaenaPLC()
    # RealSense depth camera
    camera = DracaenaCamera(max_distance=0.9)

    images = []
    rotation_counter = 0
    detection_count = 0
    waiting_image = np.zeros((1920, 1080), np.uint8)

    try:
        plc.connect_to_plc_socket()

        if use_robot:
            robot = DracaenaRobot(plc=plc)
        if use_vision:
            waiting_image = cv.imread('./images/status_waiting.png')

            if full_screen:
                cv.namedWindow(window_name, cv.WINDOW_NORMAL)
                cv.setWindowProperty(window_name, cv.WND_PROP_FULLSCREEN, cv.WINDOW_FULLSCREEN)
            else:
                cv.namedWindow(window_name, cv.WINDOW_AUTOSIZE)
                waiting_image = cv.resize(waiting_image, (1280, 720))
            cv.imshow(window_name, waiting_image)
            cv.waitKey(1)

        while True:
            print('Waiting for start signal from PLC...')
            received_data = plc.receive(2)
            if received_data == DracaenaCodes.VisionStart.value:
                camera_data = camera.get_images()

                if use_vision:
                    dv = DracaenaVision(camera_data['colour'], camera_data['depth'], window_name=window_name,
                                        debug=False)
                    print('Started vision...')
                    pixel_coordinates = dv.get_coordinates()

                    detection_count += len(pixel_coordinates)
                    images.append(dv.get_detection_image())

                    for coordinate in pixel_coordinates:
                        robot_coordinate = translate_coordinates_45cm(coordinate, [dv.centre_x, dv.centre_y])

                        x = robot_coordinate[0]
                        y = robot_coordinate[1]
                        z = robot_coordinate[2]
                        tool_angle = robot_coordinate[3]
                        centre_x = robot_coordinate[4]
                        centre_y = robot_coordinate[5]

                        print('Attempting to move robot to: X: {} Y: {} Z: {}'.format(x, y, z))
                        if use_robot:
                            robot.remove_leaf(x, y, z, tool_angle, centre_x, centre_y)
                            time.sleep(1)

                    if len(pixel_coordinates) > 0:
                        plc.send(DracaenaCodes.RotatePlant.value)
                    else:
                        plc.send(DracaenaCodes.NoDetections.value)

                    rotation_counter += 1
                    if rotation_counter >= 4:
                        display_overview_image(images, detection_count)
                        rotation_counter = 0
                        detection_count = 0
                        images = []
            elif received_data == DracaenaCodes.Shutdown.value:
                exit(0)
            elif received_data == DracaenaCodes.EmergencyStop.value:
                if use_robot:
                    robot.abort_kill_all()
                    robot.motor_power_off()
            else:
                print(received_data)
    finally:
        print('Exiting...')
        cv.destroyAllWindows()
        camera.exit()
        if use_robot:
            robot.exit()


def display_overview_image(images, detection_count):
    if len(images) != 4:
        print('Could not show overview, incorrect number of images')
        return
    horizontal_images = [np.hstack((images[0], images[1])), np.hstack((images[2], images[3]))]
    overview = np.vstack((horizontal_images[0], horizontal_images[1]))
    overview = cv.rectangle(overview, (0, 0), (1000, 80), (255, 255, 255), cv.FILLED)
    cv.putText(overview, "Number of leaves detected: {}".format(detection_count), (10, 60), cv.FONT_HERSHEY_SIMPLEX,
               2, (0, 0, 0), 5)
    if full_screen:
        resized_overview = cv.resize(overview, (1920, 1080))
    else:
        resized_overview = cv.resize(overview, (1280, 720))
    cv.imshow(window_name, resized_overview)
    cv.waitKey(1)


if __name__ == '__main__':
    main()
