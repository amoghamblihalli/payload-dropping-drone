#!/usr/bin/env python

import rospy
import cv2
import numpy as np

from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

bridge = CvBridge()  # Make the bridge a global variable

def detector(frame):
    aruco_dict = cv2.aruco.Dictionary_create(4, 4)  # Create the dictionary
    parameters = cv2.aruco.DetectorParameters_create()

    gray_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    # Detect ArUco markers
    marker_corners, marker_IDs, _ = cv2.aruco.detectMarkers(gray_frame, aruco_dict, parameters=parameters)

    if marker_corners:
        for i in range(len(marker_IDs)):
            # Draw the detected markers and the centroid
            cv2.aruco.drawDetectedMarkers(frame, marker_corners)
            
            corners = marker_corners[i].squeeze().astype(int)
            centroid = np.mean(corners, axis=0).astype(int)
            
            # Draw centroid
            cv2.circle(frame, tuple(centroid), 5, (0, 0, 255), -1)

            cv2.putText(
                frame,
                f"id: {marker_IDs[i][0]}",
                (corners[0][0], corners[0][1] - 10),
                cv2.FONT_HERSHEY_PLAIN,
                1.3,
                (200, 100, 0),
                2,
                cv2.LINE_AA,
            )

            # Print centroid coordinates
            print(f"Marker ID: {marker_IDs[i][0]} Centroid coordinates: {centroid}")
            
            # Print ARUCO DETECTED
            print("ARUCO DETECTED")

    return frame

def callback(data):
    try:
        cv_image = bridge.imgmsg_to_cv2(data, desired_encoding="bgr8")
    except CvBridgeError as e:
        rospy.logerr(e)
        return
    
    # Run ArUco marker detection on the received frame
    frame_with_aruco = detector(cv_image)

    resized_image = cv2.resize(frame_with_aruco, (360, 640)) 

    cv2.imshow("Camera output normal", frame_with_aruco)
    cv2.imshow("Camera output resized", resized_image)

    cv2.waitKey(3)

def cam_test():
    rospy.init_node('camera_read', anonymous=False)
    image_sub = rospy.Subscriber("/webcam/image_raw", Image, callback)

    try:
        rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo("Shutting down")
    
    cv2.destroyAllWindows()

if __name__ == '__main__':
    cam_test()
