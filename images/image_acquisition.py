import pyrealsense2 as rs
import numpy as np
import cv2
from pyexiv2 import Image


def main():
    # Create a pipeline
    pipeline = rs.pipeline()

    # Create a config and configure the pipeline to stream
    #  different resolutions of color and depth streams
    config = rs.config()
    config.enable_stream(rs.stream.depth, 1280, 720, rs.format.z16, 6)
    config.enable_stream(rs.stream.color, 1280, 720, rs.format.bgr8, 6)

    # Start streaming
    pipeline.start(config)

    image_counter = 0

    # Streaming loop
    try:
        while True:
            # Get frameset of color and depth
            frames = pipeline.wait_for_frames()

            # Get frames
            depth_frame = frames.get_depth_frame()
            color_frame = frames.get_color_frame()

            # Validate that both frames are valid
            if not color_frame or not depth_frame:
                continue

            depth_image = np.asanyarray(depth_frame.get_data())
            color_image = np.asanyarray(color_frame.get_data())

            # Create mask of area that is of interest
            mask_pts = np.array(
                [[576, 415], [800, 420], [1105, 630], [720, 685], [285, 590]],
                np.int32).reshape((-1, 1, 2))

            cv2.polylines(color_image, [mask_pts], True, (0, 255, 0), thickness=3)

            # Resize the colour image to make it not take up the entire screen
            # resized_image = cv2.resize(color_image, (0, 0), fx=0.5, fy=0.5)

            # Render images
            cv2.namedWindow('RealSense Stream', cv2.WINDOW_AUTOSIZE)
            cv2.imshow('RealSense Stream', color_image)
            key = cv2.waitKey(1)

            # Press esc or 'q' to close the image window
            if key & 0xFF == ord('q') or key == 27:
                cv2.destroyAllWindows()
                break

            if key & 0xFF == ord(' '):
                # Create an empty array with float values and calculate the distance for each pixel
                distarray = np.empty([720, 1280], dtype=np.float)
                for y in range(720):
                    for x in range(1280):
                        distance = depth_frame.get_distance(x, y)
                        distarray[y, x] = distance if distance < 0.9 else 0.9

                max_dist = np.max(distarray)

                # Create normalize function and apply this function to the array with distances
                normalize = lambda t: t / max_dist * 65535  # 16 bit int!
                vfunc = np.vectorize(normalize)
                normalized_distarray = vfunc(distarray)

                # Convert the normalized image with floats to uint16 to get rid of the decimals
                depth_greyscale = normalized_distarray.astype(np.uint16)

                blue, green, red = cv2.split(color_image)
                hsv = cv2.cvtColor(color_image, cv2.COLOR_BGR2HSV)
                hue, saturation, value = cv2.split(hsv)

                cv2.imwrite('image_' + str(image_counter) + '_depth-grey.png', depth_greyscale)
                cv2.imwrite('image_' + str(image_counter) + '_colour.png', color_image)
                # cv2.imwrite('image_' + str(image_counter) + '_blue.png', blue)
                # cv2.imwrite('image_' + str(image_counter) + '_green.png', green)
                # cv2.imwrite('image_' + str(image_counter) + '_red.png', red)
                # cv2.imwrite('image_' + str(image_counter) + '_hue.png', hue)
                # cv2.imwrite('image_' + str(image_counter) + '_saturation.png', saturation)
                # cv2.imwrite('image_' + str(image_counter) + '_value.png', value)

                with Image('image_' + str(image_counter) + '_depth-grey.png') as img:
                    img.modify_xmp({'Xmp.dc.max_dist': f"{max_dist}"})

                image_counter += 1
    finally:
        pipeline.stop()


if __name__ == '__main__':
    main()
