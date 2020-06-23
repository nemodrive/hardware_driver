import cv2
from typing import Optional
from vimba import *

GRID_SIZE = (7, 10)


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
        try:
            cam.GVSPAdjustPacketSize.run()

            while not cam.GVSPAdjustPacketSize.is_done():
                pass

        except (AttributeError, VimbaFeatureError):
            pass

        cam.set_pixel_format(cam.get_pixel_formats()[1])

        print(cam.get_pixel_format())


def main():

    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

    cam_id = None

    crt_img = 0

    with Vimba.get_instance():
        with get_camera(cam_id) as cam:
            setup_camera(cam)

            # Acquire 10 frame with a custom timeout (default is 2000ms) per frame acquisition.
            for frame in cam.get_frame_generator(limit=None, timeout_ms=2000):

                try:
                    ndarray_frame = frame.as_numpy_ndarray()
                except:
                    # TODO weird random error "parameter -7 invalid"
                    continue

                image = cv2.cvtColor(ndarray_frame, cv2.COLOR_BAYER_RG2RGB)

                small_img = cv2.resize(image, (int(image.shape[1] / 2), int(image.shape[0] / 2)))

                cv2.imshow("Cam View", small_img)
                cv2.waitKey(1)

                gray = cv2.cvtColor(small_img, cv2.COLOR_RGB2GRAY)

                # Find the chess board corners
                ret, corners = cv2.findChessboardCorners(gray, (GRID_SIZE[1], GRID_SIZE[0]), None)

                # If found, add object points, image points (after refining them)
                if ret:
                    corners2 = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)

                    # Draw and display the corners
                    img = cv2.drawChessboardCorners(small_img, (GRID_SIZE[1], GRID_SIZE[0]), corners2, ret)
                    cv2.imshow('corners', img)
                    key = cv2.waitKey(20)

                    if key == 27:
                        print("Saved!")
                        cv2.imwrite("./calibration_images/calib_%d.png" % crt_img, image)
                        crt_img += 1

                else:
                    print("No corners found!")


if __name__ == '__main__':
    main()
