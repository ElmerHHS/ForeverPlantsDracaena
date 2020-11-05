#!/usr/bin/env python
import numpy as np
from DracaenaVision import DracaenaVision
import pyrealsense2 as rs
import keyboard
from kawapai.robot import KawaBot
from kawa_calibration import translate_coordinates_45cm, translate_coordinates_35cm
import time
import cv2 as cv
from pyexiv2 import Image

# Set to false if no robot is connected to the PC
move_robot = True
# Set to false to only take colour + depth picture and not process them
process_vision = True
# Set to true to read from existing images
from_files = False


# Main workflow
def main():
    # Initialise variables
    robot = None
    max_distance = 0.9  # Maximum distance from the camera to the bottom is 90cm
    image_counter = 0

    # Create a pipeline
    pipeline = rs.pipeline()
    align = None
    depth_scale = 0.001

    if not from_files:
        # Create a config and configure the pipeline to stream
        #  different resolutions of color and depth streams
        config = rs.config()
        config.enable_stream(rs.stream.color, 1280, 720, rs.format.bgr8, 6)
        config.enable_stream(rs.stream.depth, 1280, 720, rs.format.z16, 6)

        # Start streaming
        profile = pipeline.start(config)

        depth_sensor = profile.get_device().first_depth_sensor()
        depth_scale = depth_sensor.get_depth_scale()
        print('Depth scale: {}'.format(depth_scale))

        # Create an align object to line up the depth frame with the colour frame
        align_to = rs.stream.color
        align = rs.align(align_to)
    else:
        color_image = cv.imread('./images/image_0_colour.png')
        filtered_distance_array = np.full([720, 1280], 0.5, dtype=np.float)

    try:
        if move_robot:
            robot = KawaBot(host="192.168.0.1", port=23)
            robot.abort_kill_all()
            robot.motor_power_off()
            robot.reset_error()
            # robot.load_as_file("./kawapai/kawasaki_side.as")
            robot.motor_power_on()
            robot.initiate_kawabot()
            robot.connect_to_movement_server()
            robot.connect_to_pose_update_server()
            print('Resetting robot position...')
            move_robot_to_home(robot)

        while True:
            print('Waiting for spacebar press to take new image...')
            key = keyboard.read_key()

            if key == 'space':
                if not from_files:
                    print('Started image acquisition...')

                    image_counter += 1

                    depth_frames = []
                    color_frame = None
                    number_of_frames = 3
                    for x in range(number_of_frames):
                        # Get a set of frames
                        frames = pipeline.wait_for_frames()

                        # Align the depth frame to color frame
                        aligned_frames = align.process(frames)

                        # Get frames
                        if x == 0:
                            color_frame = aligned_frames.get_color_frame()
                        depth_frames.append(aligned_frames.get_depth_frame())

                    # Validate that all frames are valid
                    if not color_frame or len(depth_frames) != number_of_frames:
                        continue

                    # Spatial filter
                    spatial = rs.spatial_filter()
                    spatial.set_option(rs.option.filter_magnitude, 5)
                    spatial.set_option(rs.option.filter_smooth_alpha, 1)
                    spatial.set_option(rs.option.filter_smooth_delta, 50)
                    spatial.set_option(rs.option.holes_fill, 3)
                    # Temporal filter
                    temporal = rs.temporal_filter()
                    # Hole filling filter
                    hole_filling = rs.hole_filling_filter()

                    # Initialise filtered frame variable
                    filtered_depth_frame = None
                    for x in range(number_of_frames):
                        frame = depth_frames[x]
                        frame = spatial.process(frame)
                        frame = temporal.process(frame)
                        frame = hole_filling.process(frame)
                        filtered_depth_frame = frame

                    color_image = np.asanyarray(color_frame.get_data())
                    filtered_depth_data = np.asanyarray(filtered_depth_frame.get_data())

                    # Create an empty array with float values and calculate the distance for each pixel
                    height, width = filtered_depth_data.shape
                    filtered_distance_array = np.empty([height, width], dtype=np.float)
                    for y in range(height):
                        for x in range(width):
                            filtered_distance = depth_scale * filtered_depth_data[y, x]
                            filtered_distance_array[y, x] = filtered_distance if filtered_distance <= max_distance \
                                else max_distance

                    # Create normalize function and apply this function to the array with distances
                    normalize = lambda t: t / max_distance * 65535  # 16 bit int!
                    vfunc = np.vectorize(normalize)
                    filtered_normalized_dist_array = vfunc(filtered_distance_array)

                    # Convert the normalized image with floats to uint16 to get rid of the decimals
                    filtered_normalized_depth_image = filtered_normalized_dist_array.astype(np.uint16)

                    cv.imwrite('./images/DracaenaVision/{}_depth.png'.format(image_counter), filtered_normalized_depth_image)
                    cv.imwrite('./images/DracaenaVision/{}_colour.png'.format(image_counter), color_image)

                    with Image('./images/DracaenaVision/{}_depth.png'.format(image_counter)) as img:
                        img.modify_xmp({'Xmp.dc.max_dist': f"{max_distance}"})

                if process_vision:
                    maximum_depth_of_interest = 0.7
                    dv = DracaenaVision(color_image, filtered_distance_array, maximum_depth_of_interest, debug=True)
                    print('Started vision...')
                    coordinates = dv.get_coordinates()

                    for coordinate in coordinates:
                        # robot_coordinate = translate_coordinates_45cm(coordinate)
                        robot_coordinate = translate_coordinates_35cm(coordinate)

                        if move_robot:
                            print('Press space to move robot to: X: {} Y: {} Z: {}'.format(robot_coordinate[0],
                                                                                           robot_coordinate[1],
                                                                                           robot_coordinate[2]))
                            print('Tool angle: {} degrees'.format(robot_coordinate[3]))
                            print('Pixel coordinates: X: {} Y{}'.format(coordinate[0], coordinate[1]))
                            key2 = keyboard.read_key()
                            if key2 == 'space':
                                robot.jmove_cartesian_check_pos(round(robot_coordinate[0]), round(robot_coordinate[1]),
                                                                round(robot_coordinate[2] + 100), robot_coordinate[3],
                                                                90, 0)
                                robot.lmove_cartesian_check_pos(round(robot_coordinate[0]), round(robot_coordinate[1]),
                                                                round(robot_coordinate[2]), robot_coordinate[3], 90, 0)
                                robot.jmove_cartesian_check_pos(round(robot_coordinate[0]), round(robot_coordinate[1]),
                                                                round(robot_coordinate[2] + 150), robot_coordinate[3],
                                                                90, 0)
                                move_robot_to_home(robot)
                            if key2 == 'q':
                                exit(-1)
            if key == 'q':
                exit(-1)
    finally:
        if not from_files:
            pipeline.stop()

        if move_robot:
            robot.close_movement_server()
            robot.close_pose_update_server()
            robot.motor_power_off()
            robot.disconnect()


def move_robot_to_home(kawa: KawaBot):
    kawa.jmove_joint(0, -50, -105, 5, -38, 0)
    time.sleep(5)


if __name__ == '__main__':
    main()
