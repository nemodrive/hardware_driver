import sys
import statistics
import time

import yaml
import cv2
from PyQt5 import QtWidgets, uic
from PyQt5.QtCore import QThread, pyqtSignal, Qt, pyqtSlot, QByteArray
from PyQt5.QtGui import QImage, QPixmap
import pyqtgraph as pg
import json
import numpy as np

from streamer import SharedMemStreamer


class StreamThread(QThread):

    signal_change_pixmap_left = pyqtSignal(QImage)
    signal_change_pixmap_center = pyqtSignal(QImage)
    signal_change_pixmap_right = pyqtSignal(QImage)
    signal_fps = pyqtSignal(int)
    signal_ms = pyqtSignal(int)
    signal_telemetry_text = pyqtSignal(str)

    signal_imu = pyqtSignal(dict)

    def __init__(self):
        super(StreamThread, self).__init__()

        self._is_running = True

        self.telemetry_delay_frames = 10

        # self.change_pixmap = pyqtSignal(QImage) THIS IS WRONG! Because of the internal implementation of QtSignal

    def img_ocv_to_qt(self, ocv_img):
        rgb_frame = cv2.cvtColor(ocv_img, cv2.COLOR_BGR2RGB)

        h, w, ch = rgb_frame.shape
        bytes_per_line = ch * w
        qt_image = QImage(rgb_frame.data, w, h, bytes_per_line, QImage.Format_RGB888)

        return qt_image.copy()  # TODO del this to avoid leaks

    def run(self):

        streamer = SharedMemStreamer()
        # TODO give it a warmup period?
        source_stream = streamer.stream_generator()

        telemetry_delay = self.telemetry_delay_frames + 1
        multiple_delay_ms = []

        last_time = time.time()

        debug_time = time.time()

        while self._is_running:

            recv_obj = next(source_stream)

            print("delay_recv = ", time.time() - debug_time)
            debug_time = time.time()

            # show telemetry to user


            print(recv_obj["sensor_data"]["gps"])
            print(recv_obj["sensor_data"]["canbus"])

            for pos in recv_obj["images"].keys():
                recv_obj["images"][pos] = cv2.resize(recv_obj["images"][pos],
                                                     (int(recv_obj["images"][pos].shape[1] / 2.8),
                                                      int(recv_obj["images"][pos].shape[0] / 2.8)))

            # TODO always check key exists!!! also in streamer
            try:
                self.signal_change_pixmap_left.emit(self.img_ocv_to_qt(recv_obj["images"]["left"]))
            except:
                pass

            try:
                self.signal_change_pixmap_center.emit(self.img_ocv_to_qt(recv_obj["images"]["center"]))
            except:
                pass

            try:
                self.signal_change_pixmap_right.emit(self.img_ocv_to_qt(recv_obj["images"]["right"]))
            except:
                pass

            delay = time.time() - last_time
            last_time = time.time()

            multiple_delay_ms.append(delay * 1000)

            try:
                self.signal_imu.emit(recv_obj["sensor_data"]["imu"])
            except:
                pass

            try:
                self.signal_telemetry_text.emit(json.dumps(recv_obj["sensor_data"], indent=4))
            except:
                pass

            if telemetry_delay > self.telemetry_delay_frames:

                telemetry_delay = 0

                avg_delay_ms = statistics.mean(multiple_delay_ms)
                multiple_delay_ms.clear()
                self.signal_fps.emit(int(1/avg_delay_ms * 1000))
                self.signal_ms.emit(int(avg_delay_ms))
            else:
                telemetry_delay += 1

            print("delay_gui = ", time.time() - debug_time)
            debug_time = time.time()

    def stop(self):
        self._is_running = False


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

    @pyqtSlot(int)
    def set_fps(self, fps):
        self.lcd_fps.display(fps)

    @pyqtSlot(int)
    def set_delay(self, ms):
        self.lcd_delay.display(ms)

    @pyqtSlot(str)
    def set_telemetry_text(self, text):
        self.plain_text_edit_telemetry.setPlainText(text)

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

    def start_stream(self):
        self.stream_thread = StreamThread()

        self.stream_thread.signal_change_pixmap_left.connect(self.set_pixmap_left)
        self.stream_thread.signal_change_pixmap_center.connect(self.set_pixmap_center)
        self.stream_thread.signal_change_pixmap_right.connect(self.set_pixmap_right)

        self.stream_thread.signal_fps.connect(self.set_fps)
        self.stream_thread.signal_ms.connect(self.set_delay)

        self.stream_thread.signal_telemetry_text.connect(self.set_telemetry_text)

        self.stream_thread.signal_imu.connect(self.update_imu_plot)

        self.stream_thread.start()

    def __init__(self):
        # pg.setConfigOption('background', 'w')
        # pg.setConfigOption('foreground', 'k')
        # pg.setConfigOption('leftButtonPan', False)

        super(MyWindow, self).__init__()
        uic.loadUi('gui/previewer.ui', self)

        self.stream_label_left = self.findChild(QtWidgets.QLabel, 'labelStreamLeft')
        self.stream_label_center = self.findChild(QtWidgets.QLabel, 'labelStreamCenter')
        self.stream_label_right = self.findChild(QtWidgets.QLabel, 'labelStreamRight')

        self.button_record = self.findChild(QtWidgets.QPushButton, 'pushButtonRecord')
        self.button_record.clicked.connect(self.on_click_rec)

        self.button_stop = self.findChild(QtWidgets.QPushButton, 'pushButtonStop')
        self.button_stop.clicked.connect(self.on_click_stop)
        self.button_stop.setEnabled(False)

        self.lcd_fps = self.findChild(QtWidgets.QLCDNumber, 'lcdFPS')
        self.lcd_delay = self.findChild(QtWidgets.QLCDNumber, 'lcdDelay')

        self.plain_text_edit_telemetry = self.findChild(QtWidgets.QPlainTextEdit, 'plainTextEditTelemetry')

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

        self.show()

    @pyqtSlot()
    def on_click_rec(self):
        self.start_stream()
        self.button_stop.setEnabled(True)
        self.button_record.setEnabled(False)

    @pyqtSlot()
    def on_click_stop(self):
        self.stream_thread.stop()
        self.button_stop.setEnabled(False)
        self.button_record.setEnabled(True)


if __name__ == '__main__':
    app = QtWidgets.QApplication(sys.argv)
    window = MyWindow()
    sys.exit(app.exec_())
