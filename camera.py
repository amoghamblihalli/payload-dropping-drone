import cv2 as cv
import numpy as np

# Load the Aruco dictionary (DICT_4X4_50 is used for demonstration)
def detector():
    aruco_dict = cv.aruco.Dictionary_get(cv.aruco.DICT_4X4_50)
    parameters = cv.aruco.DetectorParameters_create()

    cap = cv.VideoCapture(0)

    while True:
        ret, frame = cap.read()
        if not ret:
            break

        gray_frame = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)

        # Detect Aruco markers
        marker_corners, marker_IDs, _ = cv.aruco.detectMarkers(
            gray_frame, aruco_dict, parameters=parameters
        )

        if marker_corners:
            for i in range(len(marker_IDs)):
                # Draw the detected markers and the centroid
                cv.aruco.drawDetectedMarkers(frame, marker_corners)
                
                corners = marker_corners[i].squeeze().astype(int)
                centroid = np.mean(corners, axis=0).astype(int)
                
                # Draw centroid
                cv.circle(frame, tuple(centroid), 5, (0, 0, 255), -1)

                cv.putText(
                    frame,
                    f"id: {marker_IDs[i][0]}",
                    (corners[0][0], corners[0][1] - 10),
                    cv.FONT_HERSHEY_PLAIN,
                    1.3,
                    (200, 100, 0),
                    2,
                    cv.LINE_AA,
                )

                # Print centroid coordinates
                print(f"Marker ID: {marker_IDs[i][0]} Centroid coordinates: {centroid}")

        cv.imshow("frame", frame)
        key = cv.waitKey(1)
        if key == ord("q"):
            break

    cap.release()
    cv.destroyAllWindows()
