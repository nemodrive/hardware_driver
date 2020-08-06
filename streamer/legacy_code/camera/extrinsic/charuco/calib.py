import cv2
import yaml
from glob import glob
import numpy as np
import pickle

allCorners = []
allIds = []

criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 100, 0.00001)

aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
board = cv2.aruco.CharucoBoard_create(11, 8, .035, .026, aruco_dict)

cv2.imshow('board', board.draw((420, 297)))
cv2.waitKey(0)

for f in glob("./extra_calib/*.JPG"):
    img = cv2.imread(f)
    gray = cv2.cvtColor(img, cv2.COLOR_RGB2GRAY)

    corners, ids, rejectedImgPoints = cv2.aruco.detectMarkers(gray, aruco_dict)

    if len(corners) > 0:

        num_corners, corners2, ids2 = cv2.aruco.interpolateCornersCharuco(corners, ids, gray, board)

        if corners2 is not None and ids is not None:
            allCorners.append(corners2)
            allIds.append(ids2)

        # cv2.aruco.drawDetectedMarkers(gray, corners, ids)

        cv2.aruco.drawDetectedCornersCharuco(gray, corners2, ids2, (0, 0, 255))

        frame_small = cv2.resize(gray, (int(gray.shape[1] / 5), int(gray.shape[0] / 5)))
        cv2.imshow('sample', frame_small)
        cv2.waitKey(10)

    else:
        print("No corners found!")

# TODO check these
cameraMatrixInit = np.array([[1000.,    0., img.shape[0]/2.],
                             [   0., 1000., img.shape[1]/2.],
                             [   0.,    0.,               1.]])

distCoeffsInit = np.zeros((5, 1))
#flags = (cv2.CALIB_USE_INTRINSIC_GUESS + cv2.CALIB_RATIONAL_MODEL + cv2.CALIB_FIX_ASPECT_RATIO)
flags = (cv2.CALIB_RATIONAL_MODEL + cv2.CALIB_FIX_ASPECT_RATIO)

retCalibCharuco = cv2.aruco.calibrateCameraCharucoExtended(
                      charucoCorners=allCorners,
                      charucoIds=allIds,
                      board=board,
                      imageSize=(img.shape[1], img.shape[0]),
                      cameraMatrix=cameraMatrixInit,
                      distCoeffs=distCoeffsInit,
                      flags=flags,
                      criteria=(cv2.TERM_CRITERIA_EPS & cv2.TERM_CRITERIA_COUNT, 10000, 1e-9)
                  )

#retCalibCharuco = cv2.aruco.calibrateCameraCharuco(allCorners, allIds, board, imsize,None,None)

calib_results = {
    "ret": retCalibCharuco[0],
    "cam_matrix":  retCalibCharuco[1],
    "distortion_coeffs":  retCalibCharuco[2],
    "rvecs": retCalibCharuco[3],
    "tvecs": retCalibCharuco[4],
    "std_dev_intrinsics": retCalibCharuco[5],
    "std_dev_extrinsics": retCalibCharuco[6],
    "per_view_errors": retCalibCharuco[7],
}

with open("cam.calib", "wb") as f:
    pickle.dump(calib_results, f)

# test calib


with open("measurements.yaml", "r") as f:
    calib_samples = yaml.load(f, Loader=yaml.SafeLoader)

for f, _ in calib_samples.items():
    img = cv2.imread(f)

    h, w = img.shape[:2]
    newcameramtx, roi = cv2.getOptimalNewCameraMatrix(calib_results['cam_matrix'], calib_results['distortion_coeffs'], (w, h), 1, (w, h)) # TODO alpha param instead of crop

    corrected = img.copy()

    # undistort
    cv2.undistort(img, calib_results['cam_matrix'], calib_results['distortion_coeffs'], corrected, newcameramtx)
    # crop the image
    x, y, w, h = roi
    corrected = corrected[y:y + h, x:x + w]

    small_img = cv2.resize(corrected, (int(corrected.shape[1] / 5), int(corrected.shape[0] / 5)))

    cv2.imshow("Corrected", small_img)
    cv2.waitKey(0)
