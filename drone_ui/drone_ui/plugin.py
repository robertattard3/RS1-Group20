import sys
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, String
from std_srvs.srv import SetBool

from qt_gui.plugin import Plugin
from python_qt_binding.QtCore import QTimer
from python_qt_binding.QtWidgets import (
    QWidget, QVBoxLayout, QPushButton, QHBoxLayout, QLabel, QComboBox
)


class DroneControlPlugin(Plugin):
    """
    rqt plugin:
      - 🚨 E-STOP -> /drone/mission(SetBool){data:false}   (+ publishes /emergency_stop=True)
      - ⬆️ Takeoff -> /drone/mission(SetBool){data:true}
      - Speed selector -> /search_speed (String: 'slow'|'fast')
    """

    def __init__(self, context):
        super().__init__(context)
        self.setObjectName('DroneControlPlugin')

        # rclpy init (safe if called multiple times)
        try:
            rclpy.init(args=None)
        except Exception:
            pass

        self.node = rclpy.create_node('drone_ui')

        # Publishers
        self.estop_pub = self.node.create_publisher(Bool, '/emergency_stop', 10)
        self.speed_pub = self.node.create_publisher(String, '/search_speed', 10)

        # Service client
        self.mission_client = self.node.create_client(SetBool, '/drone/mission')
        self._pending_future = None

        # ---- UI ----
        self._widget = QWidget()
        self._widget.setWindowTitle('Drone Controls')
        layout = QVBoxLayout(self._widget)

        # Buttons (no Land button)
        self.btn_estop = QPushButton('🚨 E-STOP')
        self.btn_takeoff = QPushButton('⬆️ Takeoff')

        # styling
        self.btn_estop.setStyleSheet('font-weight: bold; padding:12px; background:#ff6666;')
        self.btn_takeoff.setStyleSheet('padding:10px;')

        layout.addWidget(self.btn_estop)
        layout.addWidget(self.btn_takeoff)

        # Speed selector
        speed_row = QHBoxLayout()
        speed_row.addWidget(QLabel('Search speed:'))
        self.speed_combo = QComboBox()
        self.speed_combo.addItems(['slow', 'fast'])
        self.speed_combo.currentIndexChanged.connect(self._on_speed_changed)
        speed_row.addWidget(self.speed_combo)
        layout.addLayout(speed_row)

        # Status line
        self.status_lbl = QLabel('Waiting for /drone/mission...')
        layout.addWidget(self.status_lbl)

        # Wire up signals
        self.btn_estop.clicked.connect(self._on_estop)
        self.btn_takeoff.clicked.connect(self._on_takeoff)

        # Spin rclpy in Qt loop & poll service
        self.timer = QTimer(self._widget)
        self.timer.timeout.connect(self._spin_once)
        self.timer.start(20)

        # dock in rqt
        if context is not None:
            context.add_widget(self._widget)

        # publish initial speed immediately
        self._publish_speed(self.speed_combo.currentText())
        self._update_service_status(first=True)

    # ---------- UI callbacks ----------

    def _on_estop(self):
        # Optional: still publish a Bool so any safety mux can latch zero velocity
        self.estop_pub.publish(Bool(data=True))
        self._call_mission(False)  # stop mission

    def _on_takeoff(self):
        self._call_mission(True)   # start mission

    def _on_speed_changed(self, _idx):
        self._publish_speed(self.speed_combo.currentText())

    # ---------- Helpers ----------

    def _publish_speed(self, label: str):
        self.speed_pub.publish(String(data=label))
        self.node.get_logger().info(f'Search speed: {label}')

    def _call_mission(self, start: bool):
        if not self.mission_client.service_is_ready():
            self.node.get_logger().warn('/drone/mission not available yet')
            self._update_service_status()
            return
        if self._pending_future is not None:
            self.node.get_logger().info('Mission call already in progress...')
            return

        req = SetBool.Request()
        req.data = bool(start)
        self._disable_ui(True)
        self.status_lbl.setText(f'Calling /drone/mission: {"start" if start else "stop"}...')
        self._pending_future = self.mission_client.call_async(req)

    def _update_service_status(self, first: bool = False):
        ready = self.mission_client.service_is_ready()
        if ready:
            self.status_lbl.setText('Service ready: /drone/mission (SetBool)')
            self._disable_ui(False)
            if first:
                self.node.get_logger().info('Connected to /drone/mission')
        else:
            self.status_lbl.setText('Waiting for /drone/mission...')
            self._disable_ui(True)

    def _disable_ui(self, disable: bool):
        self.btn_estop.setEnabled(not disable)
        self.btn_takeoff.setEnabled(not disable)
        self.speed_combo.setEnabled(not disable)

    # ---------- rclpy integration ----------

    def _spin_once(self):
        # spin callbacks
        rclpy.spin_once(self.node, timeout_sec=0.0)

        # poll service availability
        self._update_service_status()

        # handle async service result
        if self._pending_future and self._pending_future.done():
            try:
                resp = self._pending_future.result()
                if resp.success:
                    self.status_lbl.setText(f'/drone/mission ok: {resp.message}')
                    self.node.get_logger().info(f'/drone/mission success: {resp.message}')
                else:
                    self.status_lbl.setText(f'/drone/mission failed: {resp.message}')
                    self.node.get_logger().warn(f'/drone/mission failed: {resp.message}')
            except Exception as e:
                self.status_lbl.setText(f'/drone/mission error: {e}')
                self.node.get_logger().error(f'/drone/mission exception: {e}')
            finally:
                self._pending_future = None
                self._disable_ui(False)

    # ---------- rqt lifecycle ----------

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
