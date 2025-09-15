import sys
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, String

from qt_gui.plugin import Plugin
from python_qt_binding.QtCore import QTimer
from python_qt_binding.QtWidgets import (
    QWidget, QVBoxLayout, QPushButton, QHBoxLayout, QLabel, QComboBox
)

class DroneControlPlugin(Plugin):
    """rqt plugin that publishes E-STOP / Takeoff / Land commands + speed profile"""

    def __init__(self, context):
        super().__init__(context)
        self.setObjectName('DroneControlPlugin')

        # Ensure rclpy is initialized (safe to call multiple times)
        try:
            rclpy.init(args=None)
        except Exception:
            pass

        self.node = rclpy.create_node('drone_ui')

        # Publishers
        self.estop_pub   = self.node.create_publisher(Bool,   '/emergency_stop', 10)
        self.cmd_pub     = self.node.create_publisher(String, '/drone_command',  10)
        self.speed_pub   = self.node.create_publisher(String, '/search_speed',   10)  # <-- NEW

        # ---- UI ----
        self._widget = QWidget()
        self._widget.setWindowTitle('Drone Controls')
        layout = QVBoxLayout(self._widget)

        # Buttons
        self.btn_estop   = QPushButton('E-STOP')
        self.btn_takeoff = QPushButton('⬆️ Takeoff')
        self.btn_land    = QPushButton('⬇️ Land')

        self.btn_estop.setStyleSheet('font-weight: bold; padding:12px; background:#ff6666;')
        self.btn_takeoff.setStyleSheet('padding:10px;')
        self.btn_land.setStyleSheet('padding:10px;')

        layout.addWidget(self.btn_estop)
        layout.addWidget(self.btn_takeoff)
        layout.addWidget(self.btn_land)

        # Speed selector (slow / fast)
        speed_row = QHBoxLayout()
        speed_row.addWidget(QLabel('Search speed:'))
        self.speed_combo = QComboBox()
        self.speed_combo.addItems(['slow', 'fast'])
        self.speed_combo.currentIndexChanged.connect(self._on_speed_changed)
        speed_row.addWidget(self.speed_combo)
        layout.addLayout(speed_row)

        # Wire up signals
        self.btn_estop.clicked.connect(self._on_estop)
        self.btn_takeoff.clicked.connect(lambda: self._send_cmd('takeoff'))
        self.btn_land.clicked.connect(lambda: self._send_cmd('land'))

        # Spin rclpy in the Qt event loop
        self.timer = QTimer(self._widget)
        self.timer.timeout.connect(self._spin_once)
        self.timer.start(10)

        if context is not None:
            context.add_widget(self._widget)  # dock inside rqt

        # Publish initial speed
        self._publish_speed(self.speed_combo.currentText())

    def _spin_once(self):
        if self.node is not None:
            rclpy.spin_once(self.node, timeout_sec=0.0)

    def _on_estop(self):
        self.estop_pub.publish(Bool(data=True))
        self.node.get_logger().info('E-STOP pressed')

    def _send_cmd(self, text):
        self.cmd_pub.publish(String(data=text))
        self.node.get_logger().info(f'Command: {text}')

    def _on_speed_changed(self, _idx):
        self._publish_speed(self.speed_combo.currentText())

    def _publish_speed(self, label):
        self.speed_pub.publish(String(data=label))
        self.node.get_logger().info(f'Search speed: {label}')

    # rqt lifecycle hooks
    def shutdown_plugin(self):
        try:
            self.timer.stop()
        except Exception:
            pass
        if self.node is not None:
            self.node.destroy_node()

def standalone_main():
    from python_qt_binding.QtWidgets import QApplication, QWidget, QVBoxLayout, QLabel
    app = QApplication(sys.argv)
    w = QWidget(); w.setWindowTitle('Drone UI (standalone)')
    lay = QVBoxLayout(w); lay.addWidget(QLabel('Open rqt to load DroneControlPlugin'))
    w.show()
    return app.exec()
