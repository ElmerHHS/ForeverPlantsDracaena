import math
from datetime import datetime
import numpy as np
from plantcv import plantcv as pcv
import cv2 as cv


class DracaenaVision:

    def __init__(self, colour_image, depth_array, window_name: str = None, full_screen=False, debug=False):
        self.debug = debug
        self.window_name = window_name
        self.full_screen = full_screen
        self.colour_image = colour_image
        self.depth_array = depth_array
        self.blue, self.green, self.red = cv.split(self.colour_image)
        self.centre_y = 360
        self.centre_x = 640
        self.detection_image = None

        if self.debug:
            cv.imwrite('./images/DracaenaVision/input.png', self.colour_image)
            cv.imwrite('./images/DracaenaVision/blue.png', self.blue)
            cv.imwrite('./images/DracaenaVision/green.png', self.green)
            cv.imwrite('./images/DracaenaVision/red.png', self.red)

    def get_coordinates(self):
        if self.debug:
            pcv.params.debug = 'print'  # set debug mode
            pcv.params.debug_outdir = './images/DracaenaVision/'  # set output directory

        blue_threshold = pcv.threshold.binary(gray_img=self.blue, threshold=50, max_value=255, object_type='dark')
        pcv.apply_mask(self.colour_image, mask=blue_threshold, mask_color='white')

        # Calculate moments of binary image
        moments = cv.moments(blue_threshold)

        # Calculate x,y coordinate of center
        self.centre_x = int(moments["m10"] / moments["m00"])
        self.centre_y = int(moments["m01"] / moments["m00"])

        # Put text and highlight the center
        cv.circle(self.colour_image, (self.centre_x, self.centre_y), 5, (255, 255, 255), -1)
        cv.putText(self.colour_image, "Centre", (self.centre_x - 25, self.centre_y - 25), cv.FONT_HERSHEY_SIMPLEX, 0.5,
                   (255, 255, 255), 2)

        red_threshold = pcv.threshold.binary(gray_img=self.red, threshold=70, max_value=255, object_type='light')

        # Erode/Dilate the red threshold to remove noise
        red_threshold = pcv.erode(red_threshold, ksize=5, i=1)
        red_threshold = pcv.dilate(red_threshold, ksize=5, i=2)

        # Create mask of area that is of interest
        mask_pts = np.array(
            [[576, 415], [800, 420], [1105, 630], [720, 685], [285, 590]],
            np.int32).reshape((-1, 1, 2))
        area_of_interest = np.zeros(self.colour_image.shape[:2], np.uint8)
        cv.fillPoly(area_of_interest, [mask_pts], (255, 255, 255))

        red_threshold_in_area_of_interest = pcv.logical_and(area_of_interest, red_threshold)
        pcv.apply_mask(img=self.colour_image, mask=red_threshold_in_area_of_interest, mask_color='black')

        params = cv.SimpleBlobDetector_Params()
        # Unused filters
        params.filterByCircularity = False
        params.filterByConvexity = False
        params.filterByInertia = False
        # Area filter
        params.filterByArea = True
        params.maxArea = 50000
        params.minArea = 100
        # Colour filter
        params.filterByColor = True
        params.blobColor = 255
        # Misc options
        params.minDistBetweenBlobs = 100

        blob_detector = cv.SimpleBlobDetector_create(params)

        keypoints = blob_detector.detect(red_threshold, mask=area_of_interest)
        keypoints.sort(reverse=True, key=lambda kp: kp.size)

        now = datetime.now().strftime('%d-%m %H %M %S')

        im_with_keypoints = cv.drawKeypoints(self.colour_image, keypoints, np.array([]), (0, 0, 255),
                                             cv.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)

        self.detection_image = im_with_keypoints.copy()
        if self.full_screen:
            self.detection_image = cv.resize(self.detection_image, (1920, 1080))
        cv.imshow(self.window_name, self.detection_image)
        cv.waitKey(1)

        cv.polylines(im_with_keypoints, [mask_pts], True, (0, 255, 0), thickness=3)
        cv.imwrite('./images/DracaenaVision/{} detections.png'.format(now), im_with_keypoints)

        coordinates = []

        for keypoint in keypoints:
            x = keypoint.pt[0]
            y = keypoint.pt[1]

            # For each X and Y determine depth (z-coordinate)
            depth = -1
            step_size_to_centre_x = (self.centre_x - x) / 100
            step_size_to_centre_y = (self.centre_y - y) / 100

            for i in range(21):
                delta_x = step_size_to_centre_x * i
                delta_y = step_size_to_centre_y * i
                new_x = round(x + delta_x)
                new_y = round(y + delta_y)
                depth = self.depth_array[new_y, new_x]
                if depth < 0.55:
                    print('Depth found with delta ({},{})'.format(delta_x, delta_y))
                    print('Depth of: {} at {} X, {} Y'.format(depth, new_x, new_y))
                    break

            z = depth if depth < 0.55 or depth <= 0 else 0.55

            # Determine the angle at which the tool should be held towards the plant
            angle = 180 - math.degrees(math.atan2(y - self.centre_y, x - self.centre_x))

            coordinate = [x, y, z, angle]
            coordinates.append(coordinate)

        return coordinates

    def get_detection_image(self):
        return self.detection_image
