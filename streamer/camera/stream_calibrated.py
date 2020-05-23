import cv2
import pickle
from typing import Optional
from vimba import *


# FRAME_SIZE = ()


def get_camera(camera_id: Optional[str]) -> Camera:
    with Vimba.get_instance() as vimba:
        if camera_id:
            try:
                return vimba.get_camera_by_id(camera_id)

            except VimbaCameraError:
                raise Exception('Failed to access Camera \'{}\'. Abort.'.format(camera_id))

        else:
            cams = vimba.get_all_cameras()
            if not cams:
                raise Exception('No Cameras accessible. Abort.')

            return cams[0]


def setup_camera(cam: Camera):
    with cam:
        # Try to adjust GeV packet size. This Feature is only available for GigE - Cameras.
        # try:
        #     cam.GVSPAdjustPacketSize.run()
        #
        #     while not cam.GVSPAdjustPacketSize.is_done():
        #         pass
        #
        # except (AttributeError, VimbaFeatureError):
        #     pass

        cam.set_pixel_format(cam.get_pixel_formats()[1])

        print(cam.get_pixel_format())


def main():

    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

    cam_id = None

    crt_img = 0

    with open("calib_coefs_cam.pkl", "rb") as f:
        calib_data = pickle.load(f)
        print(calib_data)

    with Vimba.get_instance():
        with get_camera(cam_id) as cam:
            setup_camera(cam)

            # Acquire 10 frame with a custom timeout (default is 2000ms) per frame acquisition.
            for frame in cam.get_frame_generator(limit=None, timeout_ms=5000):

                try:
                    ndarray_frame = frame.as_numpy_ndarray()
                except:
                    # TODO weird random error "parameter -7 invalid"
                    continue

                image = cv2.cvtColor(ndarray_frame, cv2.COLOR_BAYER_RG2RGB)

                small_img = cv2.resize(image, (int(image.shape[1] / 2), int(image.shape[0] / 2)))

                cv2.imshow("Cam View", small_img)
                cv2.waitKey(1)

                h, w = image.shape[:2]
                newcameramtx, roi = cv2.getOptimalNewCameraMatrix(calib_data['mtx'], calib_data['dist'], (w, h), 1, (w, h))

                corrected = image.copy()

                # undistort
                cv2.undistort(image, calib_data['mtx'], calib_data['dist'], corrected, newcameramtx)
                # crop the image
                x, y, w, h = roi
                corrected = corrected[y:y + h, x:x + w]

                small_img = cv2.resize(corrected, (int(corrected.shape[1] / 2), int(corrected.shape[0] / 2)))

                cv2.imshow("Corrected", small_img)
                cv2.waitKey(1)

                # mapx, mapy = cv2.initUndistortRectifyMap(calib_data['mtx'], calib_data['dist'], None, newcameramtx, (w, h), 5)
                # corrected = cv2.remap(image, mapx, mapy, cv2.INTER_LINEAR)
                #
                # x, y, w, h = roi
                # corrected = corrected[y:y+h, x: x+w]
                #
                # cv2.imshow("Corrected remapped", corrected)
                # cv2.waitKey(1)

                # mean_error = 0
                #
                # for i in range(len(calib_data['objpoints'])):
                #     imgpoints2, _ = cv2.projectPoints(calib_data['objpoints'][i], calib_data['rvecs'][i], calib_data['tvecs'][i], calib_data['mtx'], calib_data['dist'])
                #     error = cv2.norm(calib_data['imgpoints'][i], imgpoints2, cv2.NORM_L2) / len(imgpoints2)
                #     mean_error += error
                #
                # print(mean_error/len(calib_data['objpoints']))


if __name__ == '__main__':
    main()
