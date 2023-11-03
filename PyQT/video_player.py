from PyQt5 import QtCore, QtGui, QtWidgets
from PyQt5.QtGui import QImage, QPixmap
from PyQt5.QtCore import QThread, QThreadPool, QRunnable, pyqtSignal, pyqtSlot, Qt
import cv2
import socket
import pickle
import struct


class VideoThread(QThread):
    image_signal = pyqtSignal(QImage)
    text_signal = pyqtSignal(str)
    HOST = '127.0.0.1'
    PORT = 8089

    def __init__(self):
        super().__init__()
        self.ThreadActive = False

    def set_socket(self):
        try:
            server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.text_signal.emit("Socket Created")
            server_socket.bind((self.HOST, self.PORT))
            self.text_signal.emit("Socket Bound")
            server_socket.listen(10)
            self.text_signal.emit("Socket Listening")

            client_socket, client_address = server_socket.accept()
            self.text_signal.emit("Accepted Client Connection")
            data = b''
            payload_size = struct.calcsize("=L")
            return server_socket, client_socket, data, payload_size
        except Exception as e:
            self.text_signal.emit("Socket Setup Error")
            return None, None, None, None

    def run(self):
        self.server_socket, self.client_socket, self.data, self.payload_size = self.set_socket()
        self.ThreadActive = True
        self.text_signal.emit("Video Running")
        while self.ThreadActive and self.client_socket is not None:
            while len(self.data) < self.payload_size:
                self.data += self.client_socket.recv(4096)
            packed_msg_size = self.data[:self.payload_size]

            self.data = self.data[self.payload_size:]
            msg_size = struct.unpack("=L", packed_msg_size)[0]

            while len(self.data) < msg_size:
                self.data += self.client_socket.recv(4096)
            frame_data = self.data[:msg_size]
            self.data = self.data[msg_size:]
            frame = pickle.loads(frame_data)
            final_image = QImage(frame.data, frame.shape[1], frame.shape[0], QImage.Format_RGB888)
            self.image_signal.emit(final_image)


    def stop(self):
        self.text_signal.emit("Video Stopping")
        self.ThreadActive = False
        self.wait()
