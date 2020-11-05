import pyrealsense2 as rs
from datetime import datetime
import numpy as np
import cv2 as cv


class DracaenaCamera:
    def __init__(self, max_distance):
        # Set maximum depth distance
        self.max_distance = max_distance

        # Create a pipeline
        self.pipeline = rs.pipeline()

        # Create a config and configure the pipeline to stream
        #  different resolutions of color and depth streams
        config = rs.config()
        config.enable_stream(rs.stream.color, 1280, 720, rs.format.bgr8, 6)
        config.enable_stream(rs.stream.depth, 1280, 720, rs.format.z16, 6)

        # Start streaming
        profile = self.pipeline.start(config)

        depth_sensor = profile.get_device().first_depth_sensor()
        self.depth_scale = depth_sensor.get_depth_scale()
        print('Depth scale: {}'.format(self.depth_scale))

        # Create an align object to line up the depth frame with the colour frame
        align_to = rs.stream.color
        self.align = rs.align(align_to)

    def get_images(self):
        print('Started image acquisition...')

        depth_frames = []
        color_frame = None
        number_of_frames = 1  # The number of depth frames to get
        while True:
            for x in range(number_of_frames):
                # Get a set of frames
                frames = self.pipeline.wait_for_frames()

                # Align the depth frame to color frame
                aligned_frames = self.align.process(frames)

                # Get frames
                if x == 0:
                    color_frame = aligned_frames.get_color_frame()
                depth_frames.append(aligned_frames.get_depth_frame())

            # Validate that all frames are valid
            if color_frame and len(depth_frames) == number_of_frames:
                break

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
                filtered_distance = self.depth_scale * filtered_depth_data[y, x]
                filtered_distance_array[y, x] = filtered_distance if filtered_distance <= self.max_distance \
                    else self.max_distance

        # Create normalize function and apply this function to the array with distances
        normalize = lambda t: t / self.max_distance * 65535  # 16 bit int!
        vfunc = np.vectorize(normalize)
        filtered_normalized_dist_array = vfunc(filtered_distance_array)

        # Convert the normalized image with floats to uint16 to get rid of the decimals
        filtered_normalized_depth_image = filtered_normalized_dist_array.astype(np.uint16)

        now = datetime.now().strftime('%d-%m %H %M %S')

        cv.imwrite('./images/DracaenaVision/{} depth.png'.format(now), filtered_normalized_depth_image)
        cv.imwrite('./images/DracaenaVision/{} colour.png'.format(now), color_image)

        return {
            'colour': color_image,
            'depth': filtered_distance_array
        }

    def exit(self):
        self.pipeline.stop()
