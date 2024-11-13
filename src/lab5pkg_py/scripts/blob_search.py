#!/usr/bin/env python

import cv2
import numpy as np

# ========================= Student's code starts here =========================

# Params for camera calibration
theta = 0.0
beta = 75 / 0.1
tx = 329
ty = 237

# Function that converts image coord to world coord
def IMG2W(col, row):
    x = (col - tx) / beta
    y = (row - ty) / beta
    return x, y


# ========================= Student's code ends here ===========================

def blob_search(image_raw, color):

    # Setup SimpleBlobDetector parameters.
    params = cv2.SimpleBlobDetector_Params()

    # ========================= Student's code starts here =========================

    # Filter by Color
    params.filterByColor = False

    # Filter by Area.
    params.filterByArea = True
    params.minArea = 50

    # Filter by Circularity
    params.filterByCircularity = False

    # Filter by Inerita
    params.filterByInertia = True

    # Filter by Convexity
    params.filterByConvexity = False

    # ========================= Student's code ends here ===========================

    # Create a detector with the parameters
    detector = cv2.SimpleBlobDetector_create(params)

    # Convert the image into the HSV color space
    hsv_image = cv2.cvtColor(image_raw, cv2.COLOR_BGR2HSV)

    # ========================= Student's code starts here =========================

    blue_lower = (102,50,50)     # blue lower
    blue_upper = (130,255,255)   # blue upper

    orange_lower = (4, 130, 130)
    orange_upper = (20, 255, 255)
    
    green_lower = (40, 50, 30)
    green_upper = (60, 255, 255)

    # Define a mask using the lower and upper bounds of the target color
    # masks = [
    #     cv2.inRange(hsv_image, blue_lower, blue_upper),
    #     cv2.inRange(hsv_image, orange_lower, orange_upper),
    #     cv2.inRange(hsv_image, green_lower, green_upper),
    # ]
    # combined_mask = masks[0] | masks[1] | masks[2]


    real_mask = None
    if color == 'green':
        real_mask = cv2.inRange(hsv_image, green_lower, green_upper)
    elif color == 'blue':
        real_mask = cv2.inRange(hsv_image, blue_lower, blue_upper)
    elif color == 'orange':
        real_mask = cv2.inRange(hsv_image, orange_lower, orange_upper)
    else:
        return []

    # ========================= Student's code ends here ===========================

    keypoints = detector.detect(real_mask)


    # Find blob centers in the image coordinates
    blob_image_center = []
    num_blobs = len(keypoints)
    for i in range(num_blobs):
        blob_image_center.append((keypoints[i].pt[0],keypoints[i].pt[1]))

    # ========================= Student's code starts here =========================

    # Draw the keypoints on the detected block
    im_with_keypoints = cv2.drawKeypoints(image_raw, keypoints, image_raw, flags=cv2.DRAW_MATCHES_FLAGS_DEFAULT)

    # ========================= Student's code ends here ===========================

    xw_yw = []

    if(num_blobs == 0):
        print("No block found!")
    else:
        # Convert image coordinates to global world coordinate using IM2W() function
        for i in range(num_blobs):
            xw_yw.append(IMG2W(blob_image_center[i][0], blob_image_center[i][1]))


    cv2.namedWindow("Camera View")
    cv2.imshow("Camera View", image_raw)
    cv2.namedWindow("Mask View")
    cv2.imshow("Mask View", real_mask)
    cv2.namedWindow("Keypoint View")
    cv2.imshow("Keypoint View", im_with_keypoints)

    cv2.waitKey(2)

    return xw_yw
