#create qt window
import sys
from PyQt5 import QtWidgets
from PyQt5.QtWidgets import QApplication, QWidget, QPushButton, QToolTip, QMessageBox, QDesktopWidget, QMainWindow
from PyQt5.QtGui import QIcon, QFont, QImage, QPixmap
from PyQt5.QtCore import QCoreApplication

import theme
import video_player

class MainWindow(QtWidgets.QMainWindow):
    def __init__(self, parent = None):
        super(MainWindow, self).__init__(parent=parent)
        self.init_ui()

    def init_ui(self):
        self.ui = theme.Ui_MainWindow()
        self.window = QWidget()
        self.ui.setupUi(self)
        self.init_buttons()
        

    def init_buttons(self):
        self.ui.button1.clicked.connect(self.button_clicked1)
        self.ui.button2.clicked.connect(self.button_clicked2)
        self.ui.exit.clicked.connect(self.exit_clicked)

    def start_video(self):
        self.thread = video_player.VideoThread()
        self.thread.image_signal.connect(self.update_image)
        self.thread.text_signal.connect(self.set_video_text)
        self.thread.start()

    def set_video_text(self, text):
        self.ui.video_status.setText(text)

    def update_image(self, image):
        if image is not None:
            self.ui.label.setPixmap(QPixmap.fromImage(image))

    def stop_video(self):
        self.ui.label.setText("No Feed")
        self.thread.stop()
        self.ui.video_status.setText("Video Stopped")

    def button_clicked1(self):
        self.start_video()

    def button_clicked2(self):
        self.stop_video()

    def exit_clicked(self):
        self.close()