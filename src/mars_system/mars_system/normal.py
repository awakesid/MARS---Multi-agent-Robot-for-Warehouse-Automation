import cv2
import cv2.aruco as aruco

cap = cv2.VideoCapture(0)

aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
parameters = aruco.DetectorParameters()
detector = aruco.ArucoDetector(aruco_dict, parameters)

while True:
    ret, frame = cap.read()

    if not ret:
        break

    corners, ids, _ = detector.detectMarkers(frame)

    if ids is not None:
        print("Detected IDs:", ids.flatten())