from PySide6.QtWidgets import QLabel
from PySide6.QtGui import QPainter, QColor, QPixmap, QBrush, QLinearGradient
from PySide6.QtCore import Qt, QRectF

class VideoFrame(QLabel):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setAttribute(Qt.WA_TranslucentBackground, False)
        self.setAutoFillBackground(True)
        palette = self.palette()
        palette.setColor(self.backgroundRole(), QColor(0, 0, 0, 255))
        self.setPalette(palette)

    def paintEvent(self, event):
        painter = QPainter(self)
        grad = QLinearGradient(0, 0, 0, self.height())
        grad.setColorAt(0, QColor(0, 0, 0, 0))
        painter.setBrush(QBrush(grad))
        painter.setPen(Qt.NoPen)
        painter.drawRect(self.rect())

        video_rect = QRectF(20, 100, 1280, 720)
        painter.setBrush(QColor(0, 0, 0, 0))
        painter.drawRect(video_rect)
