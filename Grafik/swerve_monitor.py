"""
Swerve Drive Monitor GUI - TOP VIEW
Menampilkan robot tampak atas dengan 4 roda swerve
Warna roda: KUNING (RPM+), UNGU (RPM-)
"""

import sys
import serial
import serial.tools.list_ports
import time
from datetime import datetime
from PyQt5.QtWidgets import *
from PyQt5.QtCore import *
from PyQt5.QtGui import *
import math

# ==================== KONFIGURASI ====================
BAUD_RATE = 115200
UPDATE_INTERVAL = 50  # ms (lebih cepat)

# ==================== WHEEL DATA CLASS ====================
class WheelData:
    def __init__(self, id):
        self.id = id
        self.angle = 0.0
        self.rpm = 0.0
        self.target_rpm = 0.0
        self.pwm_pos = 0
        self.pwm_spd = 0
        self.ready = False
        self.last_seen = 0
        self.connected = False
        self.x = 0  # Posisi X di robot (untuk top view)
        self.y = 0  # Posisi Y di robot (untuk top view)
        
    def update_from_string(self, data_str):
        """Parse data dari format: ID:angle,rpm,pwm_pos,pwm_spd,ready"""
        try:
            parts = data_str.split(',')
            if len(parts) >= 5:
                self.angle = float(parts[0])
                self.rpm = float(parts[1])
                self.pwm_pos = int(parts[2])
                self.pwm_spd = int(parts[3])
                self.ready = bool(int(parts[4]))
                self.last_seen = time.time()
                self.connected = True
                return True
        except:
            pass
        return False

# ==================== SERIAL THREAD ====================
class SerialThread(QThread):
    data_received = pyqtSignal(str)
    connection_status = pyqtSignal(bool, str)
    
    def __init__(self):
        super().__init__()
        self.serial_port = None
        self.running = False
        self.port_name = ""
        
    def connect_serial(self, port, baudrate):
        try:
            if self.serial_port and self.serial_port.is_open:
                self.serial_port.close()
            
            self.serial_port = serial.Serial(port, baudrate, timeout=1)
            self.port_name = port
            self.running = True
            self.connection_status.emit(True, f"Connected to {port}")
            return True
        except Exception as e:
            self.connection_status.emit(False, str(e))
            return False
    
    def disconnect_serial(self):
        self.running = False
        if self.serial_port and self.serial_port.is_open:
            self.serial_port.close()
        self.connection_status.emit(False, "Disconnected")
    
    def run(self):
        buffer = ""
        while self.running and self.serial_port and self.serial_port.is_open:
            try:
                if self.serial_port.in_waiting:
                    data = self.serial_port.read(self.serial_port.in_waiting).decode('utf-8', errors='ignore')
                    buffer += data
                    
                    while '\n' in buffer:
                        line, buffer = buffer.split('\n', 1)
                        line = line.strip()
                        if line:
                            self.data_received.emit(line)
                else:
                    time.sleep(0.01)
            except Exception as e:
                print(f"Serial error: {e}")
                self.running = False
                self.connection_status.emit(False, str(e))

# ==================== ROBOT TOP VIEW WIDGET ====================
class RobotTopView(QWidget):
    def __init__(self):
        super().__init__()
        self.wheels = [WheelData(i+1) for i in range(4)]
        self.setMinimumSize(600, 600)
        self.setMaximumSize(800, 800)
        
        # Set posisi roda (koordinat relatif terhadap center robot)
        # Roda 1: Depan-Kanan, Roda 2: Depan-Kiri, Roda 3: Belakang-Kiri, Roda 4: Belakang-Kanan
        self.wheels[0].x, self.wheels[0].y = 150, -150   # FL (Front-Left) - seharusnya roda 1
        self.wheels[1].x, self.wheels[1].y = -150, -150  # FR (Front-Right) - seharusnya roda 2
        self.wheels[2].x, self.wheels[2].y = -150, 150   # RR (Rear-Right) - seharusnya roda 3
        self.wheels[3].x, self.wheels[3].y = 150, 150    # RL (Rear-Left) - seharusnya roda 4
        
        self.setAutoFillBackground(True)
        self.setStyleSheet("""
            QWidget {
                background-color: #1a1a1a;
                border: 2px solid #333333;
                border-radius: 10px;
            }
        """)
        
    def update_wheel_data(self, wheel_id, data_str):
        if 1 <= wheel_id <= 4:
            self.wheels[wheel_id-1].update_from_string(data_str)
            self.update()
    
    def paintEvent(self, event):
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing)
        
        width = self.width()
        height = self.height()
        center_x = width // 2
        center_y = height // 2
        
        # Draw grid background
        painter.setPen(QPen(QColor(40, 40, 40), 1))
        for i in range(-300, 301, 50):
            painter.drawLine(center_x - 300, center_y + i, center_x + 300, center_y + i)
            painter.drawLine(center_x + i, center_y - 300, center_x + i, center_y + 300)
        
        # Draw robot body (persegi panjang)
        painter.setPen(QPen(QColor(100, 100, 100), 3))
        painter.setBrush(QBrush(QColor(60, 60, 60)))
        painter.drawRect(center_x - 200, center_y - 200, 400, 400)
        
        # Draw robot outline
        painter.setPen(QPen(QColor(0, 150, 255), 2))
        painter.drawRect(center_x - 202, center_y - 202, 404, 404)
        
        # Label arah robot
        painter.setPen(QPen(Qt.white, 1))
        painter.drawText(center_x - 10, center_y - 220, "DEPAN")
        painter.drawText(center_x - 10, center_y + 235, "BELAKANG")
        painter.drawText(center_x - 280, center_y - 10, "KIRI")
        painter.drawText(center_x + 250, center_y - 10, "KANAN")
        
        # Draw center point
        painter.setBrush(QBrush(QColor(0, 150, 255)))
        painter.setPen(QPen(Qt.white, 1))
        painter.drawEllipse(center_x - 5, center_y - 5, 10, 10)
        
        # Draw wheels
        wheel_radius = 50
        for i, wheel in enumerate(self.wheels):
            wheel_center_x = center_x + wheel.x
            wheel_center_y = center_y + wheel.y
            
            # Status connected
            if not wheel.connected:
                painter.setPen(QPen(QColor(100, 100, 100), 2))
                painter.setBrush(QBrush(QColor(50, 50, 50)))
                painter.drawEllipse(wheel_center_x - wheel_radius, 
                                   wheel_center_y - wheel_radius, 
                                   wheel_radius * 2, wheel_radius * 2)
                continue
            
            # Warna berdasarkan RPM
            if wheel.rpm < 0:
                wheel_color = QColor(255, 0, 255, 180)  # Ungu transparan
                line_color = QColor(255, 0, 255)
            else:
                wheel_color = QColor(255, 200, 0, 180)  # Kuning transparan
                line_color = QColor(255, 200, 0)
            
            # Draw wheel circle
            painter.setPen(QPen(line_color, 2))
            painter.setBrush(QBrush(wheel_color))
            painter.drawEllipse(wheel_center_x - wheel_radius, 
                               wheel_center_y - wheel_radius, 
                               wheel_radius * 2, wheel_radius * 2)
            
            # Draw wheel number
            painter.setPen(QPen(Qt.white, 1))
            painter.setFont(QFont("Arial", 12, QFont.Bold))
            painter.drawText(wheel_center_x - 5, wheel_center_y - 20, f"{wheel.id}")
            
            # Draw direction line (menunjukkan arah putaran roda)
            angle_rad = math.radians(-wheel.angle)
            line_length = wheel_radius - 10
            end_x = wheel_center_x + line_length * math.sin(angle_rad)
            end_y = wheel_center_y - line_length * math.cos(angle_rad)
            
            painter.setPen(QPen(line_color, 3))
            painter.drawLine(wheel_center_x, wheel_center_y, int(end_x), int(end_y))
            
            # Draw arrow head
            arrow_size = 12
            angle_arrow = angle_rad + math.pi/2
            
            arrow_x1 = end_x + arrow_size * math.cos(angle_arrow)
            arrow_y1 = end_y - arrow_size * math.sin(angle_arrow)
            arrow_x2 = end_x + arrow_size * math.cos(angle_arrow - math.pi)
            arrow_y2 = end_y - arrow_size * math.sin(angle_arrow - math.pi)
            
            painter.setBrush(QBrush(line_color))
            painter.setPen(QPen(line_color, 1))
            painter.drawPolygon([
                QPoint(int(end_x), int(end_y)),
                QPoint(int(arrow_x1), int(arrow_y1)),
                QPoint(int(arrow_x2), int(arrow_y2))
            ])
            
            # Draw RPM value
            painter.setPen(QPen(Qt.white, 1))
            painter.setFont(QFont("Arial", 10))
            rpm_text = f"{wheel.rpm:.0f}"
            painter.drawText(wheel_center_x - 20, wheel_center_y + 35, rpm_text)
        
        # Draw wheel labels
        painter.setPen(QPen(Qt.gray, 1))
        painter.setFont(QFont("Arial", 10))
        painter.drawText(center_x - 280, center_y - 180, "FL")
        painter.drawText(center_x + 240, center_y - 180, "FR")
        painter.drawText(center_x - 280, center_y + 200, "RL")
        painter.drawText(center_x + 240, center_y + 200, "RR")

# ==================== WHEEL DETAIL WIDGET ====================
class WheelDetailWidget(QWidget):
    def __init__(self, wheel_id):
        super().__init__()
        self.wheel_id = wheel_id
        self.data = WheelData(wheel_id)
        self.setMinimumSize(200, 150)
        
        self.setAutoFillBackground(True)
        self.setStyleSheet("""
            QWidget {
                background-color: #2b2b2b;
                border: 1px solid #3a3a3a;
                border-radius: 5px;
            }
            QLabel {
                color: #ffffff;
                background-color: transparent;
                padding: 2px;
            }
        """)
        
        layout = QVBoxLayout()
        
        # Title
        title = QLabel(f"WHEEL {wheel_id}")
        title.setStyleSheet("font-size: 14px; font-weight: bold; color: #00aaff;")
        layout.addWidget(title)
        
        # Data labels
        self.angle_label = QLabel("Angle: 0.0°")
        self.rpm_label = QLabel("RPM: 0.0")
        self.pwm_label = QLabel("PWM: 0/0")
        self.status_label = QLabel("Status: DISCONNECTED")
        
        for label in [self.angle_label, self.rpm_label, self.pwm_label, self.status_label]:
            label.setStyleSheet("font-size: 11px;")
            layout.addWidget(label)
        
        layout.addStretch()
        self.setLayout(layout)
        
    def update_data(self, data_str):
        self.data.update_from_string(data_str)
        
        # Update labels
        self.angle_label.setText(f"Angle: {self.data.angle:.1f}°")
        
        rpm_text = f"RPM: {self.data.rpm:.1f}"
        if self.data.rpm < 0:
            rpm_text += " ⬅ MUNDUR"
        else:
            rpm_text += " ➡ MAJU"
        self.rpm_label.setText(rpm_text)
        
        self.pwm_label.setText(f"PWM Pos: {self.data.pwm_pos} | Spd: {self.data.pwm_spd}")
        
        if not self.data.connected:
            self.status_label.setText("Status: DISCONNECTED")
            self.status_label.setStyleSheet("color: #ff6666; font-size: 11px;")
        elif self.data.ready:
            self.status_label.setText("Status: READY ✅")
            self.status_label.setStyleSheet("color: #66ff66; font-size: 11px;")
        else:
            self.status_label.setText("Status: MOVING ⏳")
            self.status_label.setStyleSheet("color: #ffff66; font-size: 11px;")

# ==================== MAIN WINDOW ====================
class SwerveMonitorWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.wheels = [WheelData(i+1) for i in range(4)]
        self.serial_thread = SerialThread()
        self.serial_thread.data_received.connect(self.process_serial_data)
        self.serial_thread.connection_status.connect(self.update_connection_status)
        
        self.init_ui()
        
        self.update_timer = QTimer()
        self.update_timer.timeout.connect(self.update_display)
        self.update_timer.start(UPDATE_INTERVAL)
        
    def init_ui(self):
        self.setWindowTitle("Swerve Drive Monitor - TOP VIEW")
        self.setGeometry(100, 100, 1400, 900)
        
        self.setStyleSheet("""
            QMainWindow {
                background-color: #1e1e1e;
            }
            QLabel {
                color: #ffffff;
            }
            QPushButton {
                background-color: #3a3a3a;
                color: white;
                border: 1px solid #5a5a5a;
                padding: 8px;
                border-radius: 4px;
                min-width: 80px;
            }
            QPushButton:hover {
                background-color: #4a4a4a;
            }
            QComboBox {
                background-color: #3a3a3a;
                color: white;
                border: 1px solid #5a5a5a;
                padding: 5px;
                border-radius: 4px;
            }
            QGroupBox {
                color: #ffffff;
                border: 2px solid #3a3a3a;
                border-radius: 5px;
                margin-top: 10px;
                font-weight: bold;
            }
            QGroupBox::title {
                subcontrol-origin: margin;
                left: 10px;
                padding: 0 5px 0 5px;
            }
        """)
        
        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        main_layout = QHBoxLayout(central_widget)
        
        # ===== LEFT PANEL - Robot View =====
        left_panel = QVBoxLayout()
        
        # Top bar
        top_bar = QHBoxLayout()
        title_label = QLabel("🤖 SWERVE DRIVE - TOP VIEW")
        title_label.setStyleSheet("font-size: 20px; font-weight: bold; color: #00aaff;")
        top_bar.addWidget(title_label)
        top_bar.addStretch()
        
        top_bar.addWidget(QLabel("Port:"))
        self.port_combo = QComboBox()
        self.port_combo.setMinimumWidth(150)
        self.refresh_ports()
        top_bar.addWidget(self.port_combo)
        
        self.refresh_btn = QPushButton("🔄 Refresh")
        self.refresh_btn.clicked.connect(self.refresh_ports)
        top_bar.addWidget(self.refresh_btn)
        
        self.connect_btn = QPushButton("🔌 Connect")
        self.connect_btn.clicked.connect(self.toggle_connection)
        self.connect_btn.setStyleSheet("background-color: #00aa00; font-weight: bold;")
        top_bar.addWidget(self.connect_btn)
        
        left_panel.addLayout(top_bar)
        
        # Status
        self.status_label = QLabel("Status: Disconnected")
        self.status_label.setStyleSheet("color: #ff6666;")
        left_panel.addWidget(self.status_label)
        
        # Robot view
        self.robot_view = RobotTopView()
        left_panel.addWidget(self.robot_view)
        
        # Statistics
        stats_group = QGroupBox("System Statistics")
        stats_layout = QHBoxLayout()
        
        self.stats_labels = {}
        stats_items = [
            ("Total RPM", "0.0"),
            ("Avg Angle", "0.0°"),
            ("Active", "0/4"),
            ("Mode", "IDLE")
        ]
        
        for label, value in stats_items:
            container = QVBoxLayout()
            title = QLabel(label)
            title.setStyleSheet("color: #aaaaaa;")
            value_label = QLabel(value)
            value_label.setStyleSheet("color: #00aaff; font-size: 16px; font-weight: bold;")
            value_label.setAlignment(Qt.AlignCenter)
            container.addWidget(title)
            container.addWidget(value_label)
            stats_layout.addLayout(container)
            self.stats_labels[label] = value_label
        
        stats_group.setLayout(stats_layout)
        left_panel.addWidget(stats_group)
        
        # ===== RIGHT PANEL - Wheel Details =====
        right_panel = QVBoxLayout()
        right_panel.setContentsMargins(10, 0, 0, 0)
        
        # Wheel details
        details_group = QGroupBox("Wheel Details")
        details_layout = QVBoxLayout()
        
        self.wheel_details = []
        for i in range(4):
            detail = WheelDetailWidget(i+1)
            details_layout.addWidget(detail)
            self.wheel_details.append(detail)
        
        details_layout.addStretch()
        details_group.setLayout(details_layout)
        right_panel.addWidget(details_group)
        
        # Log panel
        log_group = QGroupBox("Communication Log")
        log_layout = QVBoxLayout()
        
        self.log_text = QTextEdit()
        self.log_text.setReadOnly(True)
        self.log_text.setMaximumHeight(200)
        self.log_text.setStyleSheet("""
            background-color: #2b2b2b;
            color: #00ff00;
            font-family: 'Courier New';
            font-size: 11px;
        """)
        log_layout.addWidget(self.log_text)
        
        clear_log_btn = QPushButton("Clear Log")
        clear_log_btn.clicked.connect(lambda: self.log_text.clear())
        log_layout.addWidget(clear_log_btn)
        
        log_group.setLayout(log_layout)
        right_panel.addWidget(log_group)
        
        # Add both panels to main layout
        main_layout.addLayout(left_panel, 3)
        main_layout.addLayout(right_panel, 1)
        
        self.start_time = time.time()
    
    def refresh_ports(self):
        self.port_combo.clear()
        ports = serial.tools.list_ports.comports()
        for port in ports:
            self.port_combo.addItem(f"{port.device} - {port.description}")
        if self.port_combo.count() == 0:
            self.port_combo.addItem("No ports found")
    
    def toggle_connection(self):
        if self.connect_btn.text() == "🔌 Connect":
            port_text = self.port_combo.currentText()
            if port_text and "No ports" not in port_text:
                port = port_text.split(" - ")[0]
                if self.serial_thread.connect_serial(port, BAUD_RATE):
                    self.serial_thread.start()
                    self.connect_btn.setText("🔌 Disconnect")
                    self.connect_btn.setStyleSheet("background-color: #aa0000; font-weight: bold;")
                    self.log_message(f"Connected to {port}")
        else:
            self.serial_thread.disconnect_serial()
            self.connect_btn.setText("🔌 Connect")
            self.connect_btn.setStyleSheet("background-color: #00aa00; font-weight: bold;")
            self.log_message("Disconnected")
    
    def update_connection_status(self, connected, message):
        if connected:
            self.status_label.setText(f"Status: {message}")
            self.status_label.setStyleSheet("color: #66ff66;")
        else:
            self.status_label.setText(f"Status: {message}")
            self.status_label.setStyleSheet("color: #ff6666;")
            self.connect_btn.setText("🔌 Connect")
    
    def process_serial_data(self, line):
        self.log_message(line)
        
        if ':' in line:
            try:
                wheel_id_str, data = line.split(':', 1)
                if wheel_id_str.isdigit():
                    wheel_id = int(wheel_id_str)
                    if 1 <= wheel_id <= 4:
                        # Update robot view
                        self.robot_view.update_wheel_data(wheel_id, data)
                        
                        # Update detail widget
                        self.wheel_details[wheel_id-1].update_data(data)
            except Exception as e:
                print(f"Parse error: {e}")
    
    def update_display(self):
        # Update statistics
        connected_wheels = [w for w in self.robot_view.wheels if w.connected]
        
        if connected_wheels:
            total_rpm = sum(w.rpm for w in connected_wheels)
            avg_angle = sum(w.angle for w in connected_wheels) / len(connected_wheels)
            
            # Cek mode (maju/mundur)
            any_mundur = any(w.rpm < 0 for w in connected_wheels)
            mode = "MUNDUR" if any_mundur else "MAJU"
        else:
            total_rpm = 0
            avg_angle = 0
            mode = "IDLE"
        
        self.stats_labels["Total RPM"].setText(f"{total_rpm:.1f}")
        self.stats_labels["Avg Angle"].setText(f"{avg_angle:.1f}°")
        self.stats_labels["Active"].setText(f"{len(connected_wheels)}/4")
        self.stats_labels["Mode"].setText(mode)
        
        # Uptime
        uptime = int(time.time() - self.start_time)
        hours = uptime // 3600
        minutes = (uptime % 3600) // 60
        seconds = uptime % 60
        
        # Update window title dengan uptime
        self.setWindowTitle(f"Swerve Drive Monitor - TOP VIEW [Uptime: {hours:02d}:{minutes:02d}:{seconds:02d}]")
    
    def log_message(self, message):
        timestamp = datetime.now().strftime("%H:%M:%S.%f")[:-3]
        self.log_text.append(f"[{timestamp}] {message}")
        self.log_text.verticalScrollBar().setValue(self.log_text.verticalScrollBar().maximum())
    
    def closeEvent(self, event):
        self.serial_thread.disconnect_serial()
        self.serial_thread.wait()
        event.accept()

# ==================== MAIN ====================
if __name__ == "__main__":
    app = QApplication(sys.argv)
    app.setStyle('Fusion')
    window = SwerveMonitorWindow()
    window.show()
    sys.exit(app.exec_())