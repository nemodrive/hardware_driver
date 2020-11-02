import sys
import statistics
import time

import yaml
import cv2
from PyQt5 import QtWidgets, uic
from PyQt5.QtCore import QThread, pyqtSignal, Qt, pyqtSlot, QByteArray
from PyQt5.QtGui import QImage, QPixmap, QIcon
import pyqtgraph as pg
import json
import numpy as np
from threading import Event

from recording.recorders import Player


GPS_PLOT_ENABLED = False


class StreamThread(QThread):

    signal_change_pixmap_left = pyqtSignal(QImage)
    signal_change_pixmap_center = pyqtSignal(QImage)
    signal_change_pixmap_right = pyqtSignal(QImage)
    # signal_fps = pyqtSignal(int)
    signal_ms = pyqtSignal(int)
    signal_speed = pyqtSignal(int)
    signal_brake = pyqtSignal(int)
    signal_turn = pyqtSignal(int)
    signal_steer = pyqtSignal(int)

    signal_imu = pyqtSignal(dict)
    signal_gps_pos = pyqtSignal(dict)

    signal_progress = pyqtSignal(int)

    signal_crt_time = pyqtSignal(str)
    signal_end_time = pyqtSignal(str)

    signal_gps_num_sat = pyqtSignal(int)
    signal_gps_hdop = pyqtSignal(int)
    signal_gps_alt = pyqtSignal(int)

    can_play = Event()
    can_play.set()

    def __init__(self, rec_path):
        super(StreamThread, self).__init__()

        self.rec_path = rec_path
        self._is_running = True

        self.telemetry_delay_frames = 10

        self.player = Player(self.rec_path)
        self.player.start()

        # self.change_pixmap = pyqtSignal(QImage) THIS IS WRONG! Because of the internal implementation of QtSignal

    def img_ocv_to_qt(self, ocv_img):
        rgb_frame = cv2.cvtColor(ocv_img, cv2.COLOR_BGR2RGB)

        h, w, ch = rgb_frame.shape
        bytes_per_line = ch * w
        qt_image = QImage(rgb_frame.data, w, h, bytes_per_line, QImage.Format_RGB888)

        return qt_image.copy()  # TODO del this to avoid leaks

    @staticmethod
    def strfdelta(tdelta, fmt):
        d = {}
        d["H"], rem = divmod(tdelta.seconds, 3600)
        d["M"], d["S"] = divmod(rem, 60)
        return fmt.format(**d)

    def run(self):
        self.player.rewind()

        start_datetime = self.player.start_datetime
        end_datetime = self.player.end_datetime

        self.signal_end_time.emit(self.strfdelta(end_datetime - start_datetime, "{H:02d}:{M:02d}:{S:02d}"))

        total_num_frames = len(self.player)

        source_stream = self.player.stream_generator(loop=True)

        telemetry_delay = self.telemetry_delay_frames + 1
        multiple_delay_ms = []

        last_time = time.time()

        debug_time = time.time()

        previous_packet_datetime = None

        while self._is_running:

            recv_obj = next(source_stream)

            # print(recv_obj["datetime"])

            self.can_play.wait()

            total_elapsed_this_packet = time.time()

            # print("delay_recv = ", time.time() - debug_time)
            debug_time = time.time()

            # show telemetry to user

            for pos in recv_obj["images"].keys():

                if not recv_obj["images"][pos] is None:

                    # recv_obj["images"][pos] = cv2.imdecode(recv_obj["images"][pos])

                    recv_obj["images"][pos] = cv2.resize(recv_obj["images"][pos],
                                                         (int(recv_obj["images"][pos].shape[1] / 2.8),
                                                          int(recv_obj["images"][pos].shape[0] / 2.8)))

            if "left" in recv_obj["images"].keys() and recv_obj["images"]["left"] is not None:
                self.signal_change_pixmap_left.emit(self.img_ocv_to_qt(recv_obj["images"]["left"]))

            if "center" in recv_obj["images"].keys() and recv_obj["images"]["center"] is not None:
                self.signal_change_pixmap_center.emit(self.img_ocv_to_qt(recv_obj["images"]["center"]))

            if "right" in recv_obj["images"].keys() and recv_obj["images"]["right"] is not None:
                self.signal_change_pixmap_right.emit(self.img_ocv_to_qt(recv_obj["images"]["right"]))

            delay = time.time() - last_time
            last_time = time.time()

            multiple_delay_ms.append(delay * 1000)

            if "imu" in recv_obj["sensor_data"].keys() and recv_obj["sensor_data"]["imu"] is not None:
                self.signal_imu.emit(recv_obj["sensor_data"]["imu"])

            if "canbus" in recv_obj["sensor_data"].keys() and recv_obj["sensor_data"]["canbus"] is not None:

                if "speed" in recv_obj["sensor_data"]["canbus"].keys():
                    self.signal_speed.emit(int(recv_obj["sensor_data"]["canbus"]["speed"]["value"]))
                if "steer" in recv_obj["sensor_data"]["canbus"].keys():

                    # print(int(recv_obj["sensor_data"]["canbus"]["steer"]["value"]))
                    self.signal_steer.emit(int(recv_obj["sensor_data"]["canbus"]["steer"]["value"]))

                if "brake" in recv_obj["sensor_data"]["canbus"].keys():
                    self.signal_brake.emit(int(recv_obj["sensor_data"]["canbus"]["brake"]["value"]))
                if "signal" in recv_obj["sensor_data"]["canbus"].keys():
                    # print(recv_obj["sensor_data"]["canbus"]["signal"]["value"])
                    self.signal_turn.emit(int(recv_obj["sensor_data"]["canbus"]["signal"]["value"]))

            if "gps" in recv_obj["sensor_data"].keys() and recv_obj["sensor_data"]["gps"] is not None:
                if "GGA" in recv_obj["sensor_data"]["gps"]:

                    crt_gga = recv_obj["sensor_data"]["gps"]["GGA"]

                    self.signal_gps_hdop.emit(int(float(crt_gga.horizontal_dil) * 100))
                    self.signal_gps_num_sat.emit(int(crt_gga.num_sats))
                    self.signal_gps_alt.emit(int(crt_gga.altitude))

                    # todo plot lat lon

                    if GPS_PLOT_ENABLED:
                        self.signal_gps_pos.emit({"LAT": crt_gga.latitude, "LON": crt_gga.longitude})

            if "datetime" in recv_obj.keys():
                self.signal_crt_time.emit(self.strfdelta(recv_obj["datetime"] - start_datetime,
                                                         "{H:02d}:{M:02d}:{S:02d}"))

            crt_frame = self.player.crt_frame_index
            self.signal_progress.emit(int(crt_frame / total_num_frames * 100))

            if telemetry_delay > self.telemetry_delay_frames:

                telemetry_delay = 0

                avg_delay_ms = statistics.mean(multiple_delay_ms)
                multiple_delay_ms.clear()
                #self.signal_fps.emit(int(1/avg_delay_ms * 1000))
                self.signal_ms.emit(int(avg_delay_ms))

            else:
                telemetry_delay += 1

            # print("delay_gui = ", time.time() - debug_time)
            debug_time = time.time()

            # simulate delay

            if previous_packet_datetime is None:
                previous_packet_datetime = recv_obj['datetime']
            else:

                total_elapsed_this_packet = time.time() - total_elapsed_this_packet

                required_delay = (recv_obj['datetime'] - previous_packet_datetime).total_seconds() - total_elapsed_this_packet
                previous_packet_datetime = recv_obj['datetime']

                if 0 < required_delay < 2:
                    time.sleep(required_delay)

    def stop(self):
        self.can_play.set()
        self._is_running = False
        self.player.close() # todo prevent race conditions

    def pause(self):
        self.can_play.clear()

    def resume(self):
        self.can_play.set()

    def frame_advance(self):
        self.can_play.set()
        self.can_play.clear()

    def goto(self, percent):
        if not self.can_play.is_set():
            target_frame = int(percent * len(self.player) / 100)
            self.player.crt_frame_index = target_frame


class MyWindow(QtWidgets.QMainWindow):

    @pyqtSlot(QImage)
    def set_pixmap_left(self, image):
        self.stream_label_left.setPixmap(QPixmap.fromImage(image))
        del image

    @pyqtSlot(QImage)
    def set_pixmap_center(self, image):
        self.stream_label_center.setPixmap(QPixmap.fromImage(image))
        del image

    @pyqtSlot(QImage)
    def set_pixmap_right(self, image):
        self.stream_label_right.setPixmap(QPixmap.fromImage(image))
        del image

    # @pyqtSlot(int)
    # def set_fps(self, fps):
    #     self.lcd_fps.display(fps)

    @pyqtSlot(int)
    def set_delay(self, ms):
        self.lcd_delay.display(ms)

    @pyqtSlot(int)
    def set_speed(self, kph):
        self.lcd_speed.display(kph)

    @pyqtSlot(int)
    def set_brake(self, press):
        self.lcd_brake.display(press)

    @pyqtSlot(int)
    def set_turn(self, raw_turn_signal):
        if raw_turn_signal == 2:
            self.label_turn_left.setPixmap(self.pixmap_turn_left_active)
            self.label_turn_right.setPixmap(self.pixmap_turn_right_default)
        elif raw_turn_signal == 4:
            self.label_turn_left.setPixmap(self.pixmap_turn_left_default)
            self.label_turn_right.setPixmap(self.pixmap_turn_right_active)
        elif raw_turn_signal == 6:
            self.label_turn_left.setPixmap(self.pixmap_turn_left_active)
            self.label_turn_right.setPixmap(self.pixmap_turn_right_active)
        else:
            self.label_turn_left.setPixmap(self.pixmap_turn_left_default)
            self.label_turn_right.setPixmap(self.pixmap_turn_right_default)

    @pyqtSlot(int)
    def set_steer(self, raw_steer_angle):
        self.lcd_steer.display(raw_steer_angle)

    @pyqtSlot(int)
    def set_progress(self, progress):
        self.slider_seek.setValue(progress)

    @pyqtSlot(str)
    def set_crt_time(self, time_str):
        self.line_edit_crt_time.setText(time_str)

    @pyqtSlot(str)
    def set_end_time(self, time_str):
        self.line_edit_end_time.setText(time_str)

    @pyqtSlot(int)
    def set_gps_num_sat(self, num_sat):
        self.lcd_gps_num_sat.display(num_sat)

    @pyqtSlot(int)
    def set_gps_hdop(self, hdop):
        self.lcd_gps_hdop.display(hdop)

    @pyqtSlot(int)
    def set_gps_alt(self, alt):
        self.lcd_gps_alt.display(alt)

    @pyqtSlot(dict)
    def update_imu_plot(self, imu_data):
        # print(imu_data)

        self.imu_data_accel_x = np.concatenate((self.imu_data_accel_x[1:], [imu_data["linear_acceleration"]["x"]]))
        self.imu_data_accel_y = np.concatenate((self.imu_data_accel_y[1:], [imu_data["linear_acceleration"]["y"]]))
        self.imu_data_accel_z = np.concatenate((self.imu_data_accel_z[1:], [imu_data["linear_acceleration"]["z"]]))

        self.curve_accel_x.setData(self.imu_data_accel_x)
        self.curve_accel_y.setData(self.imu_data_accel_y)
        self.curve_accel_z.setData(self.imu_data_accel_z)

        # self.plot_widget_accel.repaint()

        self.imu_data_gyro_x = np.concatenate((self.imu_data_gyro_x[1:], [imu_data["gyro_rate"]["x"]]))
        self.imu_data_gyro_y = np.concatenate((self.imu_data_gyro_y[1:], [imu_data["gyro_rate"]["y"]]))
        self.imu_data_gyro_z = np.concatenate((self.imu_data_gyro_z[1:], [imu_data["gyro_rate"]["z"]]))

        self.curve_gyro_x.setData(self.imu_data_gyro_x)
        self.curve_gyro_y.setData(self.imu_data_gyro_y)
        self.curve_gyro_z.setData(self.imu_data_gyro_z)

        self.imu_data_orientation_w = np.concatenate((self.imu_data_orientation_w[1:], [imu_data["orientation_quaternion"]["w"]]))
        self.imu_data_orientation_x = np.concatenate((self.imu_data_orientation_x[1:], [imu_data["orientation_quaternion"]["x"]]))
        self.imu_data_orientation_y = np.concatenate((self.imu_data_orientation_y[1:], [imu_data["orientation_quaternion"]["y"]]))
        self.imu_data_orientation_z = np.concatenate((self.imu_data_orientation_z[1:], [imu_data["orientation_quaternion"]["z"]]))

        self.curve_orientation_w.setData(self.imu_data_orientation_w)
        self.curve_orientation_x.setData(self.imu_data_orientation_x)
        self.curve_orientation_y.setData(self.imu_data_orientation_y)
        self.curve_orientation_z.setData(self.imu_data_orientation_z)

    @pyqtSlot(dict)
    def update_gps_plot(self, coords):

        if self.gps_data_lat is None and self.gps_data_lon is None:
            self.gps_data_lat = np.array([coords["LAT"]])
            self.gps_data_lon = np.array([coords["LON"]])

            self.scatter_gps_pos = self.plot_item_gps.plot(self.gps_data_lon, self.gps_data_lat, pen=None, symbol='o')

        else:

            self.gps_data_lat = np.concatenate((self.gps_data_lat, [coords["LAT"]]))
            self.gps_data_lon = np.concatenate((self.gps_data_lon, [coords["LON"]]))

            self.scatter_gps_pos.setData(self.gps_data_lon, self.gps_data_lat)

    def start_stream(self, rec_path):
        self.stream_thread = StreamThread(rec_path)

        self.stream_thread.signal_change_pixmap_left.connect(self.set_pixmap_left)
        self.stream_thread.signal_change_pixmap_center.connect(self.set_pixmap_center)
        self.stream_thread.signal_change_pixmap_right.connect(self.set_pixmap_right)

        self.stream_thread.signal_turn.connect(self.set_turn)

        # self.stream_thread.signal_fps.connect(self.set_fps)
        self.stream_thread.signal_ms.connect(self.set_delay)
        self.stream_thread.signal_speed.connect(self.set_speed)
        self.stream_thread.signal_brake.connect(self.set_brake)
        self.stream_thread.signal_steer.connect(self.set_steer)

        self.stream_thread.signal_imu.connect(self.update_imu_plot)

        self.stream_thread.signal_gps_pos.connect(self.update_gps_plot)

        self.stream_thread.signal_progress.connect(self.set_progress)

        self.stream_thread.signal_crt_time.connect(self.set_crt_time)
        self.stream_thread.signal_end_time.connect(self.set_end_time)

        self.stream_thread.signal_gps_num_sat.connect(self.set_gps_num_sat)
        self.stream_thread.signal_gps_hdop.connect(self.set_gps_hdop)
        self.stream_thread.signal_gps_alt.connect(self.set_gps_alt)

        self.stream_thread.start()

    def __init__(self):
        # pg.setConfigOption('background', 'w')
        # pg.setConfigOption('foreground', 'k')
        # pg.setConfigOption('leftButtonPan', False)

        super(MyWindow, self).__init__()
        uic.loadUi('gui/player.ui', self)

        self.is_video_paused = False

        self.stream_label_left = self.findChild(QtWidgets.QLabel, 'labelStreamLeft')
        self.stream_label_center = self.findChild(QtWidgets.QLabel, 'labelStreamCenter')
        self.stream_label_right = self.findChild(QtWidgets.QLabel, 'labelStreamRight')

        self.line_edit_rec_path = self.findChild(QtWidgets.QLineEdit, 'lineEditRecPath')

        self.line_edit_crt_time = self.findChild(QtWidgets.QLineEdit, 'lineEditCrtTime')
        self.line_edit_end_time = self.findChild(QtWidgets.QLineEdit, 'lineEditEndTime')

        self.slider_seek = self.findChild(QtWidgets.QSlider, 'horizontalSliderSeek')
        self.slider_seek.valueChanged.connect(self.on_slider_value_changed)
        self.slider_seek.setEnabled(False)

        self.button_record = self.findChild(QtWidgets.QPushButton, 'pushButtonRecord')
        self.button_record.clicked.connect(self.on_click_rec)
        self.pixmap_play = QPixmap("./gui/play.svg")  # todo make paths absolute
        self.button_record.setIcon(QIcon(self.pixmap_play))

        self.button_pause = self.findChild(QtWidgets.QPushButton, 'pushButtonPause')
        self.button_pause.clicked.connect(self.on_click_pause)
        self.button_pause.setEnabled(False)
        self.pixmap_pause = QPixmap("./gui/pause.svg")  # todo make paths absolute
        self.button_pause.setIcon(QIcon(self.pixmap_pause))

        self.button_next_frame = self.findChild(QtWidgets.QPushButton, 'pushButtonNextFrame')
        self.button_next_frame.clicked.connect(self.on_click_next_frame)
        self.button_next_frame.setEnabled(False)
        self.pixmap_next_frame = QPixmap("./gui/skip_next.svg")  # todo make paths absolute
        self.button_next_frame.setIcon(QIcon(self.pixmap_next_frame))

        self.button_stop = self.findChild(QtWidgets.QPushButton, 'pushButtonStop')
        self.button_stop.clicked.connect(self.on_click_stop)
        self.button_stop.setEnabled(False)
        self.pixmap_stop = QPixmap("./gui/stop.svg")  # todo make paths absolute
        self.button_stop.setIcon(QIcon(self.pixmap_stop))

        # self.lcd_fps = self.findChild(QtWidgets.QLCDNumber, 'lcdFPS')
        self.lcd_delay = self.findChild(QtWidgets.QLCDNumber, 'lcdDelay')
        self.lcd_speed = self.findChild(QtWidgets.QLCDNumber, 'lcdSpeed')
        self.lcd_brake = self.findChild(QtWidgets.QLCDNumber, 'lcdBrake')
        self.lcd_steer = self.findChild(QtWidgets.QLCDNumber, 'lcdSteer')

        self.lcd_gps_num_sat = self.findChild(QtWidgets.QLCDNumber, 'lcdGPSSattelites')
        self.lcd_gps_hdop = self.findChild(QtWidgets.QLCDNumber, 'lcdGPSHDOP')
        self.lcd_gps_alt = self.findChild(QtWidgets.QLCDNumber, 'lcdGPSAltitude')

        self.pixmap_turn_left_active = QPixmap("./gui/arrow_left_filled.svg")  # todo make paths absolute
        self.pixmap_turn_left_default = QPixmap("./gui/arrow_left_blank.svg")  # todo make paths absolute

        self.pixmap_turn_right_active = QPixmap("./gui/arrow_right_filled.svg")  # todo make paths absolute
        self.pixmap_turn_right_default = QPixmap("./gui/arrow_right_blank.svg")  # todo make paths absolute

        self.label_turn_left = self.findChild(QtWidgets.QLabel, 'labelTurnLeft')
        self.label_turn_left.setPixmap(self.pixmap_turn_left_default)
        # self.label_turn_left.setHidden(True)
        self.label_turn_right = self.findChild(QtWidgets.QLabel, 'labelTurnRight')
        self.label_turn_right.setPixmap(self.pixmap_turn_right_default)
        # self.label_turn_right.setHidden(True)

        # self.plain_text_edit_telemetry = self.findChild(QtWidgets.QPlainTextEdit, 'plainTextEditTelemetry')

        self.plot_widget_accel = self.findChild(pg.PlotWidget, 'plotWidgetAccel')
        self.plot_widget_accel.setTitle("Corrected Linear Acceleration In Global Space (G)")
        self.plot_item_accel = self.plot_widget_accel.getPlotItem()
        self.plot_item_accel.addLegend()

        self.plot_widget_gyro = self.findChild(pg.PlotWidget, 'plotWidgetGyro')
        self.plot_widget_gyro.setTitle("Corrected Gyro Rate (radians/sec)")
        self.plot_item_gyro = self.plot_widget_gyro.getPlotItem()
        self.plot_item_gyro.addLegend()

        self.plot_widget_orientation = self.findChild(pg.PlotWidget, 'plotWidgetOrientation')
        self.plot_widget_orientation.setTitle("Tared Orientation (quaternion)")
        self.plot_item_orientation = self.plot_widget_orientation.getPlotItem()
        self.plot_item_orientation.addLegend()

        self.num_plot_points = 100

        self.imu_data_accel_x = np.zeros(self.num_plot_points)
        self.imu_data_accel_y = np.zeros(self.num_plot_points)
        self.imu_data_accel_z = np.zeros(self.num_plot_points)

        self.curve_accel_x = self.plot_item_accel.plot(self.imu_data_accel_x, pen='r', name="x")
        self.curve_accel_y = self.plot_item_accel.plot(self.imu_data_accel_y, pen='g', name="y")
        self.curve_accel_z = self.plot_item_accel.plot(self.imu_data_accel_z, pen='b', name="z")

        self.imu_data_gyro_x = np.zeros(self.num_plot_points)
        self.imu_data_gyro_y = np.zeros(self.num_plot_points)
        self.imu_data_gyro_z = np.zeros(self.num_plot_points)

        self.curve_gyro_x = self.plot_item_gyro.plot(self.imu_data_gyro_x, pen='r', name="x")
        self.curve_gyro_y = self.plot_item_gyro.plot(self.imu_data_gyro_y, pen='g', name="y")
        self.curve_gyro_z = self.plot_item_gyro.plot(self.imu_data_gyro_z, pen='b', name="z")

        self.imu_data_orientation_w = np.zeros(self.num_plot_points)
        self.imu_data_orientation_x = np.zeros(self.num_plot_points)
        self.imu_data_orientation_y = np.zeros(self.num_plot_points)
        self.imu_data_orientation_z = np.zeros(self.num_plot_points)

        self.curve_orientation_w = self.plot_item_orientation.plot(self.imu_data_orientation_x, pen='y', name="w")
        self.curve_orientation_x = self.plot_item_orientation.plot(self.imu_data_orientation_x, pen='r', name="x")
        self.curve_orientation_y = self.plot_item_orientation.plot(self.imu_data_orientation_y, pen='g', name="y")
        self.curve_orientation_z = self.plot_item_orientation.plot(self.imu_data_orientation_z, pen='b', name="z")

        self.plot_widget_gps = self.findChild(pg.PlotWidget, 'plotWidgetGPS')
        self.plot_widget_gps.setTitle("GPS RAW")
        self.plot_widget_gps.setHidden(not GPS_PLOT_ENABLED)
        self.plot_item_gps = self.plot_widget_gps.getPlotItem()

        self.gps_data_lat = None
        self.gps_data_lon = None

        self.scatter_gps_pos = None

        self.show()

    @pyqtSlot()
    def on_click_rec(self):

        if self.is_video_paused:
            self.stream_thread.resume()
            self.is_video_paused = False
        else:
            self.start_stream(self.line_edit_rec_path.text())

        self.button_pause.setEnabled(True)
        self.button_stop.setEnabled(True)
        self.button_record.setEnabled(False)
        self.button_next_frame.setEnabled(False)
        self.slider_seek.setEnabled(False)

    @pyqtSlot()
    def on_click_stop(self):
        self.stream_thread.stop()
        self.button_pause.setEnabled(False)
        self.button_stop.setEnabled(False)
        self.button_record.setEnabled(True)
        self.button_next_frame.setEnabled(False)
        self.slider_seek.setEnabled(False)

        self.is_video_paused = False

        self.gps_data_lat = None
        self.gps_data_lon = None

    @pyqtSlot()
    def on_click_next_frame(self):
        self.stream_thread.frame_advance()

    @pyqtSlot()
    def on_click_pause(self):
        self.stream_thread.pause()
        self.button_stop.setEnabled(True)
        self.button_record.setEnabled(True)
        self.button_pause.setEnabled(False)
        self.button_next_frame.setEnabled(True)
        self.slider_seek.setEnabled(True)

        self.is_video_paused = True

    @pyqtSlot()
    def on_slider_value_changed(self):
        if self.is_video_paused:
            # print(f"seeking to {self.slider_seek.value()}!")
            self.stream_thread.goto(self.slider_seek.value())


if __name__ == '__main__':
    app = QtWidgets.QApplication(sys.argv)
    window = MyWindow()
    # window.setStyleSheet("background-color: #474747;")
    sys.exit(app.exec_())
