import cv2
from glob import glob
import numpy as np
import pickle

GRID_SIZE = (7, 10)

criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

objp = np.zeros((GRID_SIZE[0] * GRID_SIZE[1], 3), np.float32)
objp[:, :2] = np.mgrid[0:GRID_SIZE[1], 0:GRID_SIZE[0]].T.reshape(-1, 2)

objpoints = []
imgpoints = []

input_files = glob("calibration_images/*.png")

for f in input_files:
    img = cv2.imread(f)
    gray = cv2.cvtColor(img, cv2.COLOR_RGB2GRAY)

    ret, corners = cv2.findChessboardCorners(gray, (GRID_SIZE[1], GRID_SIZE[0]), None)  # TODO does gridsize order matter? YES

    if ret:
        objpoints.append(objp)

        corners2 = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
        imgpoints.append(corners2)

        img = cv2.drawChessboardCorners(img, GRID_SIZE, corners2, ret)
        cv2.imshow('img', img)
        cv2.imwrite("checkerboard_corners.jpg", img)
        cv2.waitKey(0)
    else:
        print("No corners found!")

ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, gray.shape[::-1], None, None)

calib_data = {'mtx': mtx, 'dist': dist, 'rvecs': rvecs, 'tvecs': tvecs, 'objpoints': objpoints, 'imgpoints': imgpoints} # TODO struct

with open("calib_coefs_cam.pkl", "wb") as f:
    pickle.dump(calib_data, f)

for f in input_files:
    image = cv2.imread(f)

    h, w = image.shape[:2]
    newcameramtx, roi = cv2.getOptimalNewCameraMatrix(calib_data['mtx'], calib_data['dist'], (w, h), 1, (w, h))

    corrected = image.copy()

    # undistort
    cv2.undistort(image, calib_data['mtx'], calib_data['dist'], corrected, newcameramtx)
    # crop the image
    x, y, w, h = roi
    corrected = corrected[y:y + h, x:x + w]

    cv2.imwrite("calibrated_sample.jpg", corrected)
