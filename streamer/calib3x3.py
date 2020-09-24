import cv2

from camera_provider import CameraSharedMemProvider


criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 100, 0.00001)

aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
board = cv2.aruco.CharucoBoard_create(11, 8, .100, .075, aruco_dict)

cam1_id = "DEV_000F315CD617"
cam2_id = "DEV_000F315CDD5B"
cam3_id = "DEV_000F315CDD58"

file_number = 154

with CameraSharedMemProvider(cam1_id) as cam1:
	with CameraSharedMemProvider(cam2_id) as cam2:
		with CameraSharedMemProvider(cam3_id) as cam3:
			while True:

				frame1 = cam1.get_last_frame_as_ocv()
				frame2 = cam2.get_last_frame_as_ocv()
				frame3 = cam3.get_last_frame_as_ocv()

				original1 = frame1.copy()
				original2 = frame2.copy()
				original3 = frame3.copy()

				gray1 = cv2.cvtColor(frame1, cv2.COLOR_BGR2GRAY)
				gray2 = cv2.cvtColor(frame2, cv2.COLOR_BGR2GRAY)
				gray3 = cv2.cvtColor(frame3, cv2.COLOR_BGR2GRAY)

				corners1, ids1, rejectedImgPoints1 = cv2.aruco.detectMarkers(gray1, aruco_dict)
				corners2, ids2, rejectedImgPoints2 = cv2.aruco.detectMarkers(gray2, aruco_dict)
				corners3, ids3, rejectedImgPoints3 = cv2.aruco.detectMarkers(gray3, aruco_dict)

				if len(corners1) > 0 and len(corners2) > 0 and len(corners3) > 0:

					num_corners21, corners21, ids21 = cv2.aruco.interpolateCornersCharuco(corners1, ids1, gray1, board)
					cv2.aruco.drawDetectedCornersCharuco(frame1, corners21, ids21, (0, 0, 255))

					num_corners22, corners22, ids22 = cv2.aruco.interpolateCornersCharuco(corners2, ids2, gray2, board)
					cv2.aruco.drawDetectedCornersCharuco(frame2, corners22, ids22, (0, 0, 255))

					num_corners23, corners23, ids23 = cv2.aruco.interpolateCornersCharuco(corners3, ids3, gray3, board)
					cv2.aruco.drawDetectedCornersCharuco(frame3, corners23, ids23, (0, 0, 255))

					cv2.imshow('detection 1', cv2.resize(frame1, (int(frame1.shape[1] / 5), int(frame1.shape[0] / 5))))
					# cv2.imshow('detection 2', cv2.resize(frame2, (int(frame2.shape[1] / 5), int(frame2.shape[0] / 5))))
					# cv2.imshow('detection 3', cv2.resize(frame3, (int(frame3.shape[1] / 5), int(frame3.shape[0] / 5))))

					if cv2.waitKey(1) & 0xFF == ord('p'):
						fname = f"img_calib_demo_prime/{cam1_id}_{file_number}.jpg"
						cv2.imwrite(fname, original1)
						fname = f"img_calib_demo_prime/{cam2_id}_{file_number}.jpg"
						cv2.imwrite(fname, original2)
						fname = f"img_calib_demo_prime/{cam3_id}_{file_number}.jpg"
						cv2.imwrite(fname, original3)
						print(fname)
						file_number += 1
