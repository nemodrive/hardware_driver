import cv2
import yaml
import numpy as np
from scipy import interpolate

GRID_SIZE = (7, 10)

criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

objp = np.zeros((GRID_SIZE[0] * GRID_SIZE[1], 3), np.float32)
objp[:, :2] = np.mgrid[0:GRID_SIZE[1], 0:GRID_SIZE[0]].T.reshape(-1, 2)

objpoints = []
imgpoints = []

with open("measurements.yaml", "r") as f:
    calib_samples = yaml.load(f, Loader=yaml.SafeLoader)

for f, meas in calib_samples.items():
    img = cv2.imread(f)
    gray = cv2.cvtColor(img, cv2.COLOR_RGB2GRAY)

    ret, corners = cv2.findChessboardCorners(gray, (GRID_SIZE[1], GRID_SIZE[0]), None)  # TODO does gridsize order matter? YES

    if ret:
        objpoints.append(objp)

        corners2 = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)

        outer_corners = {
            "top-left": tuple(corners2[0][0]),
            "top-right": tuple(corners2[GRID_SIZE[1] - 1][0]),
            "bot-left": tuple(corners2[((GRID_SIZE[0] - 1) * GRID_SIZE[1]) + 0][0]),
            "bot-right": tuple(corners2[((GRID_SIZE[0] - 1) * GRID_SIZE[1]) + (GRID_SIZE[1] - 1)][0]),
        }

        meas_draw = img.copy()

        for corner_pos, corner_coord in outer_corners.items():

            int_coords = (int(corner_coord[0]), int(corner_coord[1]))

            cv2.circle(meas_draw, int_coords, 10, (0, 0, 255), -1)
            cv2.putText(meas_draw, str(meas[corner_pos]), int_coords, cv2.FONT_HERSHEY_SIMPLEX, 8, (255, 0, 0), 5, cv2.LINE_AA)

        small = cv2.resize(meas_draw, (int(meas_draw.shape[1] / 4), int(meas_draw.shape[0] / 4)))
        cv2.imshow('real_meas', small)
        # cv2.waitKey(0)

        # TODO bilinear interp

        x, y, z = [], [], []

        for corner_pos, corner_coord in outer_corners.items():
            x.append(corner_coord[0])
            y.append(corner_coord[1])
            z.append(meas[corner_pos])

        distance_func = interpolate.interp2d(x, y, z, kind='linear')

        for i in range(GRID_SIZE[0]):
            for j in range(GRID_SIZE[1]):

                coords = (corners2[(i * GRID_SIZE[1]) + j][0][0], corners2[(i * GRID_SIZE[1]) + j][0][1])
                int_coords = tuple(map(int, coords))

                cv2.circle(meas_draw, int_coords, 10, (0, 255, 0), -1)
                cv2.putText(meas_draw, f"{distance_func(coords[0], coords[1]).item():.2f}", int_coords, cv2.FONT_HERSHEY_SIMPLEX, 2, (0, 255, 0), 4, cv2.LINE_AA)

        small = cv2.resize(meas_draw, (int(meas_draw.shape[1] / 4), int(meas_draw.shape[0] / 4)))
        cv2.imshow('interp_meas', small)
        cv2.waitKey(0)

        imgpoints.append(corners2)

        # img = cv2.drawChessboardCorners(img, GRID_SIZE, corners2, ret)
        # cv2.imshow('img', img)
        # cv2.imwrite("checkerboard_corners.jpg", img)
        # cv2.waitKey(0)
    else:
        print("No corners found!")
