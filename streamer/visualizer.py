import sys
import statistics

import yaml
import cv2
from PyQt5 import QtWidgets, uic
from PyQt5.QtCore import QThread, pyqtSignal, Qt, pyqtSlot, QByteArray
from PyQt5.QtGui import QImage, QPixmap

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
        self.stream_thread.start()

    def __init__(self):
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
