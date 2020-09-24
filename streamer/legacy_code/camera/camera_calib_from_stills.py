import cv2
from glob import glob
import numpy as np


aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
board = cv2.aruco.CharucoBoard_create(11, 8, .035, .026, aruco_dict)

criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 100, 0.00001)

allCorners = []
allIds = []

decimator = 0

for img in glob('calib_stills/*.jpg'):

    frame = cv2.imread(img)
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    corners, ids, rejectedImgPoints = cv2.aruco.detectMarkers(gray, aruco_dict)

    if len(corners) > 0:
        # SUB PIXEL DETECTION
        for corner in corners:
            cv2.cornerSubPix(gray, corner, winSize=(3, 3), zeroZone=(-1, -1), criteria=criteria)

        res2 = cv2.aruco.interpolateCornersCharuco(corners, ids, gray, board)

        if res2[1] is not None and res2[2] is not None and len(res2[1]) > 3 and decimator % 1 == 0:
            allCorners.append(res2[1])
            allIds.append(res2[2])

    decimator += 1

    imsize = gray.shape
# return allCorners, allIds, imsize

    print("CAMERA CALIBRATION")

    cameraMatrixInit = np.array([[1000., 0., imsize[0] / 2.],
                                 [0., 1000., imsize[1] / 2.],
                                 [0., 0., 1.]])

    distCoeffsInit = np.zeros((5, 1))
    flags = (cv2.CALIB_USE_INTRINSIC_GUESS + cv2.CALIB_RATIONAL_MODEL + cv2.CALIB_FIX_ASPECT_RATIO)
    # flags = (cv2.CALIB_RATIONAL_MODEL)
    (reproj_err, mtx, dist,
     rotation_vectors, translation_vectors,
     stdDeviationsIntrinsics, stdDeviationsExtrinsics,
     perViewErrors) = cv2.aruco.calibrateCameraCharucoExtended(
        charucoCorners=allCorners,
        charucoIds=allIds,
        board=board,
        imageSize=imsize,
        cameraMatrix=cameraMatrixInit,
        distCoeffs=distCoeffsInit,
        flags=flags,
        criteria=(cv2.TERM_CRITERIA_EPS & cv2.TERM_CRITERIA_COUNT, 10000, 1e-9))

    #return ret, camera_matrix, distortion_coefficients0, rotation_vectors, translation_vectors

    print(reproj_err)

    img = cv2.imread('calib_stills/0.jpg')
    h, w = img.shape[:2]

    newcameramtx, roi = cv2.getOptimalNewCameraMatrix(mtx, dist, (w,h), 1, (w,h))

    # undistort
    dst = cv2.undistort(img, mtx, dist, None, newcameramtx)

    cv2.imshow("undistorted", dst)
    cv2.waitKey(0)

    # crop the image
    x, y, w, h = roi
    dst = dst[y:y+h, x:x+w]

    cv2.imshow("croppd", dst)
    cv2.waitKey(0)

    # # remapping to keep edges straight
    # mapx, mapy = cv.initUndistortRectifyMap(mtx, dist, None, newcameramtx, (w,h), 5)
    # dst = cv.remap(img, mapx, mapy, cv.INTER_LINEAR)
    # # crop the image
    # x, y, w, h = roi
    # dst = dst[y:y+h, x:x+w]
    # cv.imwrite('calibresult.png', dst)
