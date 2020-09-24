import cv2
import numpy as np


def show_small(name, frame, scale_factor=0.5):
    small_img = cv2.resize(frame, (int(frame.shape[1] * scale_factor), int(frame.shape[0] * scale_factor)))
    cv2.imshow(name, small_img)


criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 100, 0.00001)

aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
board = cv2.aruco.CharucoBoard_create(11, 8, .035, .026, aruco_dict)

# cv2.imshow('board', board.draw((420, 297)))
# cv2.waitKey(0)

corners_so_far = []
ids_so_far = []

file_number = 0

imsize = None
demo_frame = None

cap = cv2.VideoCapture('calibration_images/calibration_video.mp4')

while cap.isOpened():
    ret, frame = cap.read()

    if frame is None:
        break

    if imsize is None:
        imsize = frame.shape[:2]

    original = frame.copy()

    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    corners, ids, rejected_points = cv2.aruco.detectMarkers(gray, aruco_dict)

    if len(corners) > 0:

        for corner in corners:
            cv2.cornerSubPix(gray, corner, winSize=(3, 3), zeroZone=(-1, -1), criteria=criteria)

        num_corners, corners2, ids2 = cv2.aruco.interpolateCornersCharuco(corners, ids, gray, board)

        if corners2 is None or ids2 is None or len(corners2) < 3:
            print("detection error! skipping frame!")
            continue

        cv2.aruco.drawDetectedCornersCharuco(frame, corners2, ids2, (0, 0, 255))
        show_small("detection", frame, 0.8)

        if cv2.waitKey(1) & 0xFF == ord('p'):

            fname = f"calib_stills/{file_number}.jpg"
            cv2.imwrite(fname, original)
            print(fname)
            file_number += 1

            if demo_frame is None:
                demo_frame = original
                print("demo frame registered!")

            # save the fine points

            corners_so_far.append(corners2)
            ids_so_far.append(ids2)

            # compute live reprojection

            initial_mtx = np.array([[1000., 0., imsize[0] / 2.],
                                    [0., 1000., imsize[1] / 2.],
                                    [0., 0., 1.]])
            initial_dist_coefs = np.zeros((5, 1))

            flags = (cv2.CALIB_USE_INTRINSIC_GUESS + cv2.CALIB_RATIONAL_MODEL + cv2.CALIB_FIX_ASPECT_RATIO)
            # flags = (cv2.CALIB_RATIONAL_MODEL)

            (reproj_err, mtx, dist,
             rvecs, tvecs,
             stdev_intrinsics, stdev_extrinsics,
             per_view_errors) = cv2.aruco.calibrateCameraCharucoExtended(
                charucoCorners=corners_so_far,
                charucoIds=ids_so_far,
                board=board,
                imageSize=imsize,
                cameraMatrix=initial_mtx,
                distCoeffs=initial_dist_coefs,
                flags=flags,
                criteria=(cv2.TERM_CRITERIA_EPS & cv2.TERM_CRITERIA_COUNT, 10000, 1e-9))

            # ret, camera_matrix, distortion_coefficients0, rotation_vectors, translation_vectors

            print("ERROR: ", reproj_err)
            print("StDev Intrinsics: ", stdev_intrinsics)
            print("StDev Extrinsics: ", stdev_extrinsics)
            print("Per View Errors: ", per_view_errors)

            h, w = imsize

            newcameramtx, roi = cv2.getOptimalNewCameraMatrix(mtx, dist, (w, h), 1, (w, h))

            # undistort
            dst = cv2.undistort(demo_frame, mtx, dist, None, newcameramtx)

            show_small("normal demo", demo_frame, 0.3)
            show_small("undistorted demo", dst, 0.3)

            # cv2.waitKey(0)
