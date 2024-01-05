import time
import math
from threading import Thread
from inputs import get_gamepad
import ctypes
import sys
from PyQt5.QtWidgets import QApplication, QWidget, QVBoxLayout, QHBoxLayout, QProgressBar, QLabel
from PyQt5.QtGui import QPainter, QColor, QPen, QFont, QImage
from PyQt5.QtCore import Qt, QRectF, pyqtSignal, QThread

class UpdateThread(QThread):
    update_signal = pyqtSignal(str, int)

    def run(self):
        while True:
            events = get_gamepad()
            for event in events:
                #print(f'{event.code}: {event.state}')
                if event.code == 'ABS_X':
                    self.update_signal.emit(event.code, event.state)
                elif event.code == 'ABS_Y':
                    self.update_signal.emit(event.code, event.state)
                elif event.code == 'ABS_RX':
                    self.update_signal.emit(event.code, event.state)
                elif event.code == 'Key':
                    # Handle button events if needed
                    pass

class SwerveVisualizer(QWidget):
    _motor_states = [0, 0, 0, 0, 0, 0, 0, 0]

    def __init__(self):
        super().__init__()
        self.arrow = QImage('arrow.png')

    def paintEvent(self, event):
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing)

        # Draw four circles with filled rectangles in a 2 x 2 grid
        cell_width, cell_height = self.width() / 2, self.height() / 2

        lookup = [0, 1, 4, 5, 2, 3, 6, 7]

        for i in range(2):
            for j in range(2):
                # Draw a circle with a thicker border
                circle_radius = min(cell_width, cell_height) / 2.2  # Reduce the radius to add some spacing
                circle_center = QRectF(i * cell_width, j * cell_height, cell_width, cell_height)

                pen = QPen(QColor(0, 0, 0))  # Black color for the border
                pen.setWidth(3)  # Set the thickness of the line
                painter.setPen(pen)

                painter.drawEllipse(circle_center.center(), circle_radius, circle_radius)

                # Draw a colored rectangle in the middle
                rect_width = circle_radius * 2
                rect_height = 20
                temp = circle_center.center()
                rect = QRectF(temp.x() - rect_height / 2, temp.y() - circle_radius, 20, rect_width)

                # Rotate the painter before drawing the rectangle
                offset = int(((j * 2) + i) * 2)
                value = self._motor_states[offset + 1]
                painter.save()
                painter.translate(rect.center())
                painter.rotate(-value)
                painter.translate(-rect.center())
                #painter.fillRect(rect, QColor(255, 0, 0))  # Red color
                painter.drawImage(rect, self.arrow)
                painter.restore()

                # Draw text in the middle of the circle
                value = round(self._motor_states[offset], 2)
                text = f'{value}'
                font = QFont("Arial", 14)
                painter.setFont(font)
                text_rect = painter.boundingRect(circle_center, Qt.AlignCenter, text)
                painter.drawText(text_rect, Qt.AlignCenter, text)

                # Draw a circle with a thicker border
                circle_radius = min(cell_width, cell_height) / 2.2  # Reduce the radius to add some spacing
                circle_center = QRectF(i * cell_width, j * cell_height, cell_width, cell_height)

        cell_width, cell_height = self.width() / 2, self.height() / 2

        # Draw a colored rectangle in the middle
        rect_width = circle_radius * 2
        rect_height = 20
        temp = circle_center.center()
        rect = QRectF(cell_width, cell_height / 2, 20, rect_width)

        # Rotate the painter before drawing the rectangle
        value = self._motor_states[-1] * 180 / math.pi
        painter.save()
        painter.translate(rect.center())
        painter.rotate(-value)
        painter.translate(-rect.center())
        #painter.fillRect(rect, QColor(255, 0, 0))  # Red color
        painter.drawImage(rect, self.arrow)
        painter.restore()


    def update_motors(self, list):
        if list is not None:
            self._motor_states = list
            self.update()
            #print(self._motor_states)

class InputVisualizer(QWidget):

    def __init__(self):
        super().__init__()

        self.xaxis_progress = QProgressBar()
        self.yaxis_progress = QProgressBar()
        self.omega_progress = QProgressBar()
        self.xaxis = QLabel("Xaxis")
        self.yaxis = QLabel("Yaxis")
        self.omegaStr = QLabel("Omega")

        layout = QVBoxLayout()
        layout.addWidget(self.xaxis)
        layout.addWidget(self.xaxis_progress)
        layout.addWidget(self.yaxis)
        layout.addWidget(self.yaxis_progress)
        layout.addWidget(self.omegaStr)
        layout.addWidget(self.omega_progress)

        self.setLayout(layout)

    def update_inputs(self, list):
        self.xaxis_progress.setValue(list[0])
        self.yaxis_progress.setValue(list[1])
        self.omega_progress.setValue(list[2])
        self.update()

class SwerveKinematics(QThread):
    state_emitter = pyqtSignal(list)
    update_signal = pyqtSignal(list)

    def __init__(self):
        super().__init__()
        lib = ctypes.CDLL('build/libwpitest.so')

        # Define the function signature
        self.init = lib.init
        self.init.argtypes = (ctypes.c_double, ctypes.c_double)
        self.init.restype = ctypes.c_int

        self.calculate = lib.calculate
        self.calculate.argtypes = (ctypes.c_double, ctypes.c_double, ctypes.c_double, ctypes.c_double)
        self.calculate.restype = ctypes.c_int

        self.get_states = lib.get_states
        self.get_states.restype = ctypes.POINTER(ctypes.c_double)

        self.init(0.5, 0.5)

        self.robot_rotation = 0
        self.vx = 0
        self.vy = 0
        self.omega = 0

        self.update_thread = UpdateThread()
        self.update_thread.update_signal.connect(self.new_input_values)
        self.update_thread.start()

    def deadband(self, value, deadband=0.25):
        if abs(value) > deadband:
            return value
        return 0

    def new_input_values(self, ev_type, ev_value):
        temp = self.deadband(ev_value / 32768, 0.1)
        if ev_type == 'ABS_Y':
            self.vx = -temp
        elif ev_type == 'ABS_X':
            self.vy = -temp
        elif ev_type == 'ABS_RX':
            self.omega = -temp

    def get_motors(self):
        states = self.get_states()
        temp = [states[0], states[1], states[2], states[3], states[4], states[5], states[6], states[7]]
        return temp

    def scale_inputs(self):
        return [int((self.vx + 0.5) * 100), int((self.vy + 0.5) * 100), int((self.omega + 0.5) * 100)]

    def run(self):
        Ts = 0.1
        while True:
            time.sleep(Ts)
            # Update the input UI
            update_ui = self.scale_inputs()
            self.update_signal.emit(update_ui)

            rotation = self.omega * Ts
            self.robot_rotation += rotation
            if self.robot_rotation > math.pi:
                self.robot_rotation = -math.pi
            elif self.robot_rotation < -math.pi:
                self.robot_rotation = math.pi

            self.calculate(self.vx, self.vy, self.omega, self.robot_rotation)
            states = self.get_motors()
            states.append(self.robot_rotation)
            if states is not None:
                self.state_emitter.emit(states)

            print(f'States: {states}')

if __name__ == '__main__':
    app = QApplication(sys.argv)

    main_window = QWidget()
    main_layout = QVBoxLayout()

    input_widget = InputVisualizer()
    main_layout.addWidget(input_widget)

    drawing_widget = SwerveVisualizer()
    main_layout.addWidget(drawing_widget)

    kinematics = SwerveKinematics()
    kinematics.update_signal.connect(input_widget.update_inputs)
    kinematics.state_emitter.connect(drawing_widget.update_motors)
    kinematics.start()

    main_window.setLayout(main_layout)
    main_window.setWindowTitle('Swerve Drive Simulation')
    main_window.setGeometry(100, 100, 600, 600)

    main_window.show()
    sys.exit(app.exec_())
