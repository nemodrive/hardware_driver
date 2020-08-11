import sys
import statistics

import yaml
import cv2
from PyQt5 import QtWidgets, uic
from PyQt5.QtCore import QThread, pyqtSignal, Qt, pyqtSlot, QByteArray
from PyQt5.QtGui import QImage, QPixmap
import pyqtgraph as pg
import json

import numpy as np

from network.threaded_client import ThreadedBroadcastClient


class StreamThread(QThread):

    signal_change_pixmap_left = pyqtSignal(QImage)
    signal_change_pixmap_center = pyqtSignal(QImage)
    signal_change_pixmap_right = pyqtSignal(QImage)
    signal_fps = pyqtSignal(int)
    signal_ms = pyqtSignal(int)
    signal_packet_size = pyqtSignal(float)
    signal_width = pyqtSignal(int)
    signal_height = pyqtSignal(int)
    signal_link_speed = pyqtSignal(float)
    signal_telemetry_text = pyqtSignal(str)

    signal_imu = pyqtSignal(dict)

    def __init__(self, host, port):
        super(StreamThread, self).__init__()

        self.host = host
        self.port = port

        self._is_running = True

        # self.change_pixmap = pyqtSignal(QImage) THIS IS WRONG! Because of the internal implementation of QtSignal

    def img_ocv_to_qt(self, ocv_img):
        rgb_frame = cv2.cvtColor(ocv_img, cv2.COLOR_BGR2RGB)

        h, w, ch = rgb_frame.shape
        bytes_per_line = ch * w
        qt_image = QImage(rgb_frame.data, w, h, bytes_per_line, QImage.Format_RGB888)

        return qt_image.copy()  # TODO del this to avoid leaks

    def run(self):

        with open("config.yaml", "r") as f:
            settings = yaml.load(f, Loader=yaml.SafeLoader)["network_streaming"]

        with ThreadedBroadcastClient(self.host, self.port, settings["header_size"], settings["recv_buffer"]) as c:

            telemetry_delay = 6
            multiple_delay_ms = []
            multiple_packet_size = []

            while self._is_running:

                recv_obj, delay_ms, packet_size = c.recv_object_with_net_stats()

                for pos in recv_obj["images"].keys():
                    recv_obj["images"][pos] = cv2.imdecode(recv_obj["images"][pos], cv2.IMREAD_COLOR)

                # self.signal_change_pixmap_left.emit(self.img_ocv_to_qt(recv_obj["images"]["left"]))
                self.signal_change_pixmap_center.emit(self.img_ocv_to_qt(recv_obj["images"]["center"]))
                self.signal_change_pixmap_right.emit(self.img_ocv_to_qt(recv_obj["images"]["right"]))

                multiple_delay_ms.append(delay_ms)
                multiple_packet_size.append(packet_size)

                self.signal_imu.emit(recv_obj["sensor_data"]["imu"])

                self.signal_telemetry_text.emit(json.dumps(recv_obj["sensor_data"], indent=4))

                if telemetry_delay > 5:
                    telemetry_delay = 0

                    avg_delay_ms = statistics.mean(multiple_delay_ms)
                    multiple_delay_ms.clear()
                    avg_packet_size = statistics.mean(multiple_packet_size)
                    multiple_packet_size.clear()

                    self.signal_fps.emit(int(1/avg_delay_ms * 1000))
                    self.signal_ms.emit(int(avg_delay_ms))
                    self.signal_width.emit(recv_obj["images"]["center"].shape[1])
                    self.signal_height.emit(recv_obj["images"]["center"].shape[0])
                    self.signal_packet_size.emit(avg_packet_size / (1024 ** 2))
                    self.signal_link_speed.emit(((avg_packet_size * 8) / (1024 ** 2)) / (avg_delay_ms / 1000))
                else:
                    telemetry_delay += 1

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

    @pyqtSlot(float)
    def set_packet_size(self, megabytes):
        self.lcd_packet_size.display(megabytes)

    @pyqtSlot(int)
    def set_width(self, width):
        self.lcd_width.display(width)

    @pyqtSlot(int)
    def set_height(self, height):
        self.lcd_height.display(height)

    @pyqtSlot(float)
    def set_link_speed(self, speed):
        self.lcd_link_speed.display(speed)

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

    def start_stream(self, host, port):
        self.stream_thread = StreamThread(host, port)

        self.stream_thread.signal_change_pixmap_left.connect(self.set_pixmap_left)
        self.stream_thread.signal_change_pixmap_center.connect(self.set_pixmap_center)
        self.stream_thread.signal_change_pixmap_right.connect(self.set_pixmap_right)

        self.stream_thread.signal_fps.connect(self.set_fps)
        self.stream_thread.signal_ms.connect(self.set_delay)
        self.stream_thread.signal_packet_size.connect(self.set_packet_size)
        self.stream_thread.signal_width.connect(self.set_width)
        self.stream_thread.signal_height.connect(self.set_height)
        self.stream_thread.signal_link_speed.connect(self.set_link_speed)

        self.stream_thread.signal_telemetry_text.connect(self.set_telemetry_text)

        self.stream_thread.signal_imu.connect(self.update_imu_plot)

        self.stream_thread.start()

    def __init__(self):
        # pg.setConfigOption('background', 'w')
        # pg.setConfigOption('foreground', 'k')
        # pg.setConfigOption('leftButtonPan', False)

        super(MyWindow, self).__init__()
        uic.loadUi('gui/stream.ui', self)

        self.stream_label_left = self.findChild(QtWidgets.QLabel, 'labelStreamLeft')
        self.stream_label_center = self.findChild(QtWidgets.QLabel, 'labelStreamCenter')
        self.stream_label_right = self.findChild(QtWidgets.QLabel, 'labelStreamRight')

        self.line_edit_host = self.findChild(QtWidgets.QLineEdit, 'lineEditHost')
        self.line_edit_port = self.findChild(QtWidgets.QLineEdit, 'lineEditPort')

        self.button_connect = self.findChild(QtWidgets.QPushButton, 'pushButtonConnect')
        self.button_connect.clicked.connect(self.on_click_connect)

        self.button_disconnect = self.findChild(QtWidgets.QPushButton, 'pushButtonDisconnect')
        self.button_disconnect.clicked.connect(self.on_click_disconnect)
        self.button_disconnect.setEnabled(False)

        self.lcd_fps = self.findChild(QtWidgets.QLCDNumber, 'lcdFPS')
        self.lcd_delay = self.findChild(QtWidgets.QLCDNumber, 'lcdDelay')
        self.lcd_packet_size = self.findChild(QtWidgets.QLCDNumber, 'lcdPacketSize')
        self.lcd_height = self.findChild(QtWidgets.QLCDNumber, 'lcdHeight')
        self.lcd_width = self.findChild(QtWidgets.QLCDNumber, 'lcdWidth')
        self.lcd_link_speed = self.findChild(QtWidgets.QLCDNumber, 'lcdLinkSpeed')

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
    def on_click_connect(self):
        self.start_stream(self.line_edit_host.text(), int(self.line_edit_port.text()))
        self.button_disconnect.setEnabled(True)
        self.button_connect.setEnabled(False)

    @pyqtSlot()
    def on_click_disconnect(self):
        self.stream_thread.stop()
        self.button_disconnect.setEnabled(False)
        self.button_connect.setEnabled(True)


if __name__ == '__main__':
    app = QtWidgets.QApplication(sys.argv)
    window = MyWindow()
    sys.exit(app.exec_())
