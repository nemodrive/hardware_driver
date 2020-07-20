import cv2
import yaml
import pickle
import numpy as np
from scipy import interpolate


def undistort_image(img, calib_results):
    h, w = img.shape[:2]
    newcameramtx, roi = cv2.getOptimalNewCameraMatrix(calib_results['cam_matrix'], calib_results['distortion_coeffs'],
                                                      (w, h), 1, (w, h))  # TODO alpha param instead of crop

    corrected = img.copy()

    # undistort
    cv2.undistort(img, calib_results['cam_matrix'], calib_results['distortion_coeffs'], corrected, newcameramtx)
    # crop the image
    x, y, w, h = roi
    corrected = corrected[y:y + h, x:x + w]

    return corrected

    # small_img = cv2.resize(corrected, (int(corrected.shape[1] / 5), int(corrected.shape[0] / 5)))
    # cv2.imshow("Corrected", small_img)
    # cv2.waitKey(0)


def show_small(img, name="small", scale=4):
    small_frame = cv2.resize(img, (int(img.shape[1] / scale), int(img.shape[0] / scale)))
    cv2.imshow(name, small_frame)
    return small_frame


allCorners = []
allIds = []

criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 100, 0.00001)

aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
BOARD_SHAPE = (11, 8)
board = cv2.aruco.CharucoBoard_create(11, 8, .035, .026, aruco_dict)

img = board.draw((200*3, 200*3))

cv2.imshow('board', board.draw((420, 297)))
cv2.waitKey(0)

with open("measurements.yaml", "r") as f:
    calib_samples = yaml.load(f, Loader=yaml.SafeLoader)

with open("cam.calib", "rb") as f:
    calib_results = pickle.load(f)

for f, meas in calib_samples.items():
    img = cv2.imread(f)

    img = undistort_image(img, calib_results)

    meas_draw = img.copy()

    gray = cv2.cvtColor(img, cv2.COLOR_RGB2GRAY)

    corners, ids, rejectedImgPoints = cv2.aruco.detectMarkers(gray, aruco_dict)

    if len(corners) > 0:

        num_corners, corners2, ids2 = cv2.aruco.interpolateCornersCharuco(corners, ids, gray, board)

        if corners2 is not None and ids is not None:
            allCorners.append(corners2)
            allIds.append(ids2)

        # cv2.aruco.drawDetectedMarkers(gray, corners, ids)
        # cv2.aruco.drawDetectedCornersCharuco(gray, corners2, ids2, (0, 0, 255))

        corners_dict = {}

        for i in range(corners2.shape[0]):
            corners_dict[ids2[i].item()] = tuple(corners2[i][0])

        for corner_id, corner_coords in corners_dict.items():

            int_coords = (int(corner_coords[0]), int(corner_coords[1]))

            cv2.circle(img, int_coords, 10, (0, 0, 255), -1)
            cv2.putText(img, str(corner_id), int_coords, cv2.FONT_HERSHEY_SIMPLEX, 3, (255, 0, 0), 5, cv2.LINE_AA)

        show_small(img, "markers")

        outer_corners = {
            "top-left": corners_dict[60],
            "top-right": corners_dict[69],
            "bot-left": corners_dict[0],
            "bot-right": corners_dict[9],
        }

        for corner_pos, corner_coords in outer_corners.items():
            int_coords = (int(corner_coords[0]), int(corner_coords[1]) + 80)
            cv2.putText(meas_draw, str(meas[corner_pos]), int_coords, cv2.FONT_HERSHEY_SIMPLEX, 3, (0, 255, 0), 5, cv2.LINE_AA)

        show_small(meas_draw, "real")

        x, y, z = [], [], []

        for corner_pos, corner_coord in outer_corners.items():
            x.append(corner_coord[0])
            y.append(corner_coord[1])
            z.append(meas[corner_pos])

        distance_func = interpolate.interp2d(x, y, z, kind='linear')

        for point_id, point_coords in corners_dict.items():
            int_coords = (int(point_coords[0]), int(point_coords[1]))

            cv2.circle(meas_draw, int_coords, 10, (0, 0, 255), -1)
            cv2.putText(meas_draw, f"{distance_func(point_coords[0], point_coords[1]).item():.3f}", int_coords, cv2.FONT_HERSHEY_SIMPLEX, 2, (255, 0, 0), 5, cv2.LINE_AA)

        show_small(meas_draw, "interp")
        cv2.imwrite("interp.jpg", meas_draw)
        cv2.waitKey(0)

        # for row point_coords in corners_dict.items():
        #     int_coords = (int(point_coords[0]), int(point_coords[1]))
        #
        #     cv2.circle(meas_draw, int_coords, 10, (0, 0, 255), -1)
        #     cv2.putText(meas_draw, f"{distance_func(point_coords[0], point_coords[1]).item():.3f}", int_coords, cv2.FONT_HERSHEY_SIMPLEX, 2, (255, 0, 0), 5, cv2.LINE_AA)


    else:
        print("No corners found!")

cal = cv2.aruco.calibrateCameraCharuco(allCorners, allIds, board, gray.shape, None, None)
