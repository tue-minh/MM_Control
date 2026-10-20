"""
Simple PyQt6 control GUI for the TUE USB binary protocol.

Design goals:
  * Plain, non-colorful look - no custom theme, just the default Qt style.
  * One tab per subsystem, each tab lists EVERY command of that subsystem as
    its own row (CAN ID + name + button).
  * Each tab has its own "Feedback" box BELOW the command list showing the
    human-readable RESPONSE of each command - not the raw or parsed packet.

Tabs:
  * System       0x100-0x107
  * Stepper      0x200-0x21A   (CoreXY + Z)
  * Raw Stepper  0x280-0x281   (per-motor pulse counting)
  * Servo        0x400-0x415   (STS3215 bus servos)
"""

import os
import sys
from datetime import datetime
from typing import Callable, Optional

# Allow running this file directly (app.py lives in python/gui/).
current_dir = os.path.dirname(os.path.abspath(__file__))
parent_dir = os.path.dirname(current_dir)
if parent_dir not in sys.path:
    sys.path.insert(0, parent_dir)

from PyQt6.QtCore import Qt, QThread, pyqtSignal
from PyQt6.QtGui import QFont
from PyQt6.QtWidgets import (
    QApplication,
    QCheckBox,
    QComboBox,
    QDoubleSpinBox,
    QFrame,
    QGridLayout,
    QGroupBox,
    QHBoxLayout,
    QLabel,
    QLineEdit,
    QMainWindow,
    QMessageBox,
    QPlainTextEdit,
    QPushButton,
    QScrollArea,
    QSpinBox,
    QStatusBar,
    QTabWidget,
    QVBoxLayout,
    QWidget,
)

from tue_usb import (
    LedState,
    ServoMode,
    TueUsbClient,
)


class AsyncWorker(QThread):
    """Runs one blocking client call off the GUI thread."""

    result_ready = pyqtSignal(object)
    error_occurred = pyqtSignal(str)

    def __init__(self, func: Callable[[], object]):
        super().__init__()
        self._func = func

    def run(self):
        try:
            self.result_ready.emit(self._func())
        except Exception as exc:  # noqa: BLE001 - surfaced to the UI
            self.error_occurred.emit(str(exc))


class TueUsbApp(QMainWindow):
    """Connection bar + one tab per subsystem, each with its own feedback box."""

    def __init__(self):
        super().__init__()
        self.setWindowTitle("TUE USB Control")
        self.resize(1000, 720)

        self.client = TueUsbClient(mock=True)
        self._workers = []

        self._init_ui()
        self._refresh_ports()
        self._update_connection_ui()

    # ==================================================================
    # UI construction
    # ==================================================================

    def _init_ui(self):
        central = QWidget()
        self.setCentralWidget(central)
        root = QVBoxLayout(central)
        root.setContentsMargins(8, 8, 8, 8)
        root.setSpacing(8)

        root.addWidget(self._build_connection_bar())

        self.tabs = QTabWidget()
        self.tabs.addTab(self._build_system_tab(), "System")
        self.tabs.addTab(self._build_stepper_tab(), "Stepper (CoreXY + Z)")
        self.tabs.addTab(self._build_raw_stepper_tab(), "Raw Stepper")
        self.tabs.addTab(self._build_servo_tab(), "Servo (STS3215)")
        root.addWidget(self.tabs)

        self.status = QStatusBar()
        self.setStatusBar(self.status)
        self.status.showMessage("Select a port and connect, or tick 'Mock'.")

    def _build_connection_bar(self) -> QWidget:
        bar = QFrame()
        layout = QHBoxLayout(bar)
        layout.setContentsMargins(4, 2, 4, 2)
        layout.setSpacing(8)

        layout.addWidget(QLabel("Port:"))
        self.cmb_port = QComboBox()
        self.cmb_port.setMinimumWidth(130)
        layout.addWidget(self.cmb_port)

        btn_refresh = QPushButton("Refresh")
        btn_refresh.clicked.connect(self._refresh_ports)
        layout.addWidget(btn_refresh)

        layout.addWidget(QLabel("Baud:"))
        self.cmb_baud = QComboBox()
        for baud in (115200, 230400, 460800, 921600, 1000000):
            self.cmb_baud.addItem(str(baud), baud)
        layout.addWidget(self.cmb_baud)

        self.btn_connect = QPushButton("Connect")
        self.btn_connect.clicked.connect(self._toggle_connection)
        layout.addWidget(self.btn_connect)

        self.chk_mock = QCheckBox("Mock")
        self.chk_mock.setChecked(True)
        self.chk_mock.toggled.connect(self._on_mock_toggled)
        layout.addWidget(self.chk_mock)

        layout.addStretch()
        self.lbl_conn = QLabel("DISCONNECTED")
        layout.addWidget(self.lbl_conn)
        return bar

    # ------------------------------------------------------------------
    # Small layout helpers
    # ------------------------------------------------------------------

    def _grid_box(self, title: str) -> tuple[QGroupBox, QGridLayout]:
        box = QGroupBox(title)
        grid = QGridLayout(box)
        grid.setContentsMargins(8, 8, 8, 8)
        grid.setHorizontalSpacing(8)
        grid.setVerticalSpacing(6)
        return box, grid

    def _cmd_row(self, grid: QGridLayout, row: int, can_label: str,
                 buttons: list[tuple[str, Callable[[], None]]]):
        """One listed command: CAN-ID label on the left, its buttons on the right."""
        grid.addWidget(QLabel(can_label), row, 0)
        for col, (text, handler) in enumerate(buttons, start=1):
            btn = QPushButton(text)
            btn.clicked.connect(handler)
            grid.addWidget(btn, row, col)

    def _feedback_box(self, title: str) -> tuple[QGroupBox, QPlainTextEdit]:
        box = QGroupBox(title)
        box.setMinimumHeight(150)
        layout = QVBoxLayout(box)
        layout.setContentsMargins(6, 6, 6, 6)
        layout.setSpacing(4)

        top = QHBoxLayout()
        top.addWidget(QLabel("Command response (newest at the bottom):"))
        top.addStretch()
        log = QPlainTextEdit()
        log.setReadOnly(True)
        log.setFont(QFont("Consolas", 10))
        log.setMaximumBlockCount(2000)
        btn_clear = QPushButton("Clear")
        btn_clear.clicked.connect(log.clear)
        top.addWidget(btn_clear)
        layout.addLayout(top)
        layout.addWidget(log)
        return box, log

    def _assemble_tab(self, top_widgets: list[QWidget],
                      feedback_box: QWidget) -> QWidget:
        """Params + commands in a scroll area, feedback pinned at the bottom."""
        tab = QWidget()
        outer = QVBoxLayout(tab)
        outer.setContentsMargins(8, 8, 8, 8)
        outer.setSpacing(8)

        content = QWidget()
        c_layout = QVBoxLayout(content)
        c_layout.setContentsMargins(0, 0, 0, 0)
        c_layout.setSpacing(8)
        for widget in top_widgets:
            c_layout.addWidget(widget)
        c_layout.addStretch()

        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        scroll.setFrameShape(QFrame.Shape.NoFrame)
        scroll.setWidget(content)
        outer.addWidget(scroll, 1)
        outer.addWidget(feedback_box, 0)
        return tab

    def _spin(self, lo: int, hi: int, value: int, width: int = 100) -> QSpinBox:
        s = QSpinBox()
        s.setRange(lo, hi)
        s.setValue(value)
        s.setMaximumWidth(width)
        return s

    def _dspin(self, lo: float, hi: float, value: float, width: int = 100) -> QDoubleSpinBox:
        s = QDoubleSpinBox()
        s.setRange(lo, hi)
        s.setDecimals(3)
        s.setValue(value)
        s.setMaximumWidth(width)
        return s

    # ==================================================================
    # Tab: System (0x100-0x107)
    # ==================================================================

    def _build_system_tab(self) -> QWidget:
        pbox, pg = self._grid_box("Parameters")
        pg.addWidget(QLabel("Ping token:"), 0, 0)
        self.txt_ping_token = QLineEdit("0x12345678")
        self.txt_ping_token.setMaximumWidth(140)
        pg.addWidget(self.txt_ping_token, 0, 1)

        pg.addWidget(QLabel("LED index:"), 0, 2)
        self.spn_led_idx = self._spin(0, 0, 0, 60)
        pg.addWidget(self.spn_led_idx, 0, 3)

        pg.addWidget(QLabel("Reset magic:"), 1, 0)
        self.txt_reset_magic = QLineEdit("0xDEADBEEF")
        self.txt_reset_magic.setMaximumWidth(140)
        pg.addWidget(self.txt_reset_magic, 1, 1)

        pg.addWidget(QLabel("Echo data:"), 1, 2)
        self.txt_echo = QLineEdit("HELLO")
        self.txt_echo.setMaximumWidth(140)
        pg.addWidget(self.txt_echo, 1, 3)

        cbox, cg = self._grid_box("Commands")
        r = 0
        self._cmd_row(cg, r, "0x100 SYS_PING", [("Send ping", self._do_ping)]); r += 1
        self._cmd_row(cg, r, "0x101 SYS_INFO", [("Query info", self._do_info)]); r += 1
        self._cmd_row(cg, r, "0x102/103 SYS_UID", [("Query UID", self._do_uid)]); r += 1
        self._cmd_row(cg, r, "0x104 SYS_UPTIME", [("Query uptime", self._do_uptime)]); r += 1
        self._cmd_row(cg, r, "0x105 SYS_LED_CTRL", [
            ("Off", lambda: self._do_led(LedState.OFF)),
            ("On", lambda: self._do_led(LedState.ON)),
            ("Toggle", lambda: self._do_led(LedState.TOGGLE)),
            ("Query", lambda: self._do_led(LedState.QUERY)),
        ]); r += 1
        self._cmd_row(cg, r, "0x106 SYS_RESET", [("Reset MCU", self._do_reset)]); r += 1
        self._cmd_row(cg, r, "0x107 SYS_ECHO", [("Send echo", self._do_echo)]); r += 1
        cg.setColumnStretch(7, 1)

        fbox, self.log_sys = self._feedback_box("SYS Feedback")
        return self._assemble_tab([pbox, cbox], fbox)

    # ==================================================================
    # Tab: Stepper (0x200-0x21A)
    # ==================================================================

    def _build_stepper_tab(self) -> QWidget:
        pbox, pg = self._grid_box("Parameters")
        pg.addWidget(QLabel("X (mm):"), 0, 0)
        self.spn_x = self._dspin(-1000.0, 1000.0, 50.0)
        pg.addWidget(self.spn_x, 0, 1)
        pg.addWidget(QLabel("Y (mm):"), 0, 2)
        self.spn_y = self._dspin(-1000.0, 1000.0, 30.0)
        pg.addWidget(self.spn_y, 0, 3)
        pg.addWidget(QLabel("Z (mm):"), 0, 4)
        self.spn_z = self._dspin(-500.0, 500.0, 10.0)
        pg.addWidget(self.spn_z, 0, 5)

        pg.addWidget(QLabel("Feed (mm/s):"), 1, 0)
        self.spn_feed = self._spin(1, 500, 60)
        pg.addWidget(self.spn_feed, 1, 1)
        pg.addWidget(QLabel("Accel (mm/s^2):"), 1, 2)
        self.spn_accel = self._spin(1, 10000, 600)
        pg.addWidget(self.spn_accel, 1, 3)
        pg.addWidget(QLabel("Wait timeout (ms):"), 1, 4)
        self.spn_timeout = self._spin(0, 60000, 5000, 110)
        pg.addWidget(self.spn_timeout, 1, 5)

        pg.addWidget(QLabel("Jog Vx (mm/s):"), 2, 0)
        self.spn_jog_vx = self._spin(-500, 500, 10)
        pg.addWidget(self.spn_jog_vx, 2, 1)
        pg.addWidget(QLabel("Jog Vy (mm/s):"), 2, 2)
        self.spn_jog_vy = self._spin(-500, 500, 10)
        pg.addWidget(self.spn_jog_vy, 2, 3)
        pg.addWidget(QLabel("Jog Vz (mm/s):"), 2, 4)
        self.spn_jog_vz = self._spin(-500, 500, 5)
        pg.addWidget(self.spn_jog_vz, 2, 5)

        cbox, cg = self._grid_box("Commands")
        r = 0
        self._cmd_row(cg, r, "0x200 STEP_GET_INFO", [("Read config", self._do_step_info)]); r += 1
        self._cmd_row(cg, r, "0x201 STEP_GET_STATUS", [("Read status", self._do_step_status)]); r += 1
        self._cmd_row(cg, r, "0x202 STEP_ENABLE", [
            ("Enable", lambda: self._do_step_enable(True)),
            ("Disable", lambda: self._do_step_enable(False)),
            ("Query", lambda: self._do_step_enable(0xFF)),
        ]); r += 1
        self._cmd_row(cg, r, "0x203 STEP_GET_POS_XY", [("Read XY", self._do_step_pos_xy)]); r += 1
        self._cmd_row(cg, r, "0x204 STEP_GET_POS_Z", [("Read Z", self._do_step_pos_z)]); r += 1
        self._cmd_row(cg, r, "0x205 STEP_GET_TGT_XY", [("Read target XY", self._do_step_tgt_xy)]); r += 1
        self._cmd_row(cg, r, "0x206 STEP_GET_TGT_Z", [("Read target Z", self._do_step_tgt_z)]); r += 1
        self._cmd_row(cg, r, "0x207 STEP_SETPOS_XY", [("Set position XY", self._do_step_setpos_xy)]); r += 1
        self._cmd_row(cg, r, "0x208 STEP_SETPOS_Z", [("Set position Z", self._do_step_setpos_z)]); r += 1
        self._cmd_row(cg, r, "0x210/211 STEP_MOVE_COORDS+EXEC", [("Move XYZ", self._do_step_move_xyz)]); r += 1
        self._cmd_row(cg, r, "0x212 STEP_MOVE_XY", [("Move XY", self._do_step_move_xy)]); r += 1
        self._cmd_row(cg, r, "0x213 STEP_MOVE_Z", [("Move Z", self._do_step_move_z)]); r += 1
        self._cmd_row(cg, r, "0x214 STEP_MOVE_REL_XY", [("Move relative XY", self._do_step_move_rel_xy)]); r += 1
        self._cmd_row(cg, r, "0x215 STEP_SET_SPEED", [("Set default speed", self._do_step_set_speed)]); r += 1
        self._cmd_row(cg, r, "0x216 STEP_JOG_XY", [("Jog XY", self._do_step_jog_xy)]); r += 1
        self._cmd_row(cg, r, "0x217 STEP_JOG_Z", [("Jog Z", self._do_step_jog_z)]); r += 1
        self._cmd_row(cg, r, "0x218 STEP_STOP", [("Stop", self._do_step_stop)]); r += 1
        self._cmd_row(cg, r, "0x219 STEP_ABORT", [("Abort", self._do_step_abort)]); r += 1
        self._cmd_row(cg, r, "0x21A STEP_WAIT_IDLE", [("Wait idle", self._do_step_wait_idle)]); r += 1
        cg.setColumnStretch(4, 1)

        fbox, self.log_step = self._feedback_box("STEP Feedback")
        return self._assemble_tab([pbox, cbox], fbox)

    # ==================================================================
    # Tab: Raw Stepper (0x280-0x281)
    # ==================================================================

    def _build_raw_stepper_tab(self) -> QWidget:
        pbox, pg = self._grid_box("Parameters")
        pg.addWidget(QLabel("Motor:"), 0, 0)
        self.cmb_raw_motor = QComboBox()
        self.cmb_raw_motor.addItem("1 - CoreXY A", 1)
        self.cmb_raw_motor.addItem("2 - CoreXY B", 2)
        self.cmb_raw_motor.addItem("3 - Z axis", 3)
        pg.addWidget(self.cmb_raw_motor, 0, 1)
        pg.addWidget(QLabel("Steps:"), 0, 2)
        self.spn_raw_steps = self._spin(-1000000, 1000000, 1600, 120)
        pg.addWidget(self.spn_raw_steps, 0, 3)
        pg.addWidget(QLabel("Feed (mm/s):"), 0, 4)
        self.spn_raw_feed = self._spin(0, 500, 40)
        pg.addWidget(self.spn_raw_feed, 0, 5)

        cbox, cg = self._grid_box("Commands")
        self._cmd_row(cg, 0, "0x280 STEP_RAW_GET_STEPS", [("Read steps", self._do_raw_read)])
        self._cmd_row(cg, 1, "0x281 STEP_RAW_MOVE", [("Move steps", self._do_raw_move)])

        fbox, self.log_raw = self._feedback_box("RAW Feedback")
        return self._assemble_tab([pbox, cbox], fbox)

    # ==================================================================
    # Tab: Servo (0x400-0x415)
    # ==================================================================

    def _build_servo_tab(self) -> QWidget:
        pbox, pg = self._grid_box("Parameters")
        pg.addWidget(QLabel("Servo ID:"), 0, 0)
        self.spn_servo_id = self._spin(1, 254, 1)
        pg.addWidget(self.spn_servo_id, 0, 1)
        pg.addWidget(QLabel("Position (0..4095):"), 0, 2)
        self.spn_servo_pos = self._spin(0, 4095, 2048)
        pg.addWidget(self.spn_servo_pos, 0, 3)
        pg.addWidget(QLabel("Speed (steps/s):"), 0, 4)
        self.spn_servo_speed = self._spin(0, 5000, 800)
        pg.addWidget(self.spn_servo_speed, 0, 5)

        pg.addWidget(QLabel("Acceleration:"), 1, 0)
        self.spn_servo_accel = self._spin(0, 254, 30)
        pg.addWidget(self.spn_servo_accel, 1, 1)
        pg.addWidget(QLabel("PWM (-1000..1000):"), 1, 2)
        self.spn_servo_pwm = self._spin(-1000, 1000, 0)
        pg.addWidget(self.spn_servo_pwm, 1, 3)
        pg.addWidget(QLabel("Torque limit (0.1%):"), 1, 4)
        self.spn_servo_tlim = self._spin(0, 1000, 500)
        pg.addWidget(self.spn_servo_tlim, 1, 5)

        pg.addWidget(QLabel("Mode:"), 2, 0)
        self.cmb_servo_mode = QComboBox()
        for mode in (ServoMode.POSITION, ServoMode.SPEED, ServoMode.PWM, ServoMode.STEP):
            self.cmb_servo_mode.addItem(f"{mode.value} - {mode.name}", int(mode))
        pg.addWidget(self.cmb_servo_mode, 2, 1)
        pg.addWidget(QLabel("Register addr (hex):"), 2, 2)
        self.txt_servo_reg = QLineEdit("0x38")
        self.txt_servo_reg.setMaximumWidth(90)
        pg.addWidget(self.txt_servo_reg, 2, 3)
        pg.addWidget(QLabel("Read length (1..6):"), 2, 4)
        self.spn_servo_len = self._spin(1, 6, 2, 70)
        pg.addWidget(self.spn_servo_len, 2, 5)

        pg.addWidget(QLabel("Write data (hex):"), 3, 0)
        self.txt_servo_wdata = QLineEdit("")
        self.txt_servo_wdata.setPlaceholderText("e.g. 01 02 03")
        self.txt_servo_wdata.setMaximumWidth(160)
        pg.addWidget(self.txt_servo_wdata, 3, 1)
        pg.addWidget(QLabel("New ID:"), 3, 2)
        self.spn_servo_new_id = self._spin(1, 254, 2)
        pg.addWidget(self.spn_servo_new_id, 3, 3)

        cbox, cg = self._grid_box("Commands")
        r = 0
        self._cmd_row(cg, r, "0x400 SERVO_PING", [("Ping", self._do_servo_ping)]); r += 1
        self._cmd_row(cg, r, "0x401 SERVO_SET_POS", [("Set position", self._do_servo_set_pos)]); r += 1
        self._cmd_row(cg, r, "0x402 SERVO_GET_POS", [("Read position", self._do_servo_get_pos)]); r += 1
        self._cmd_row(cg, r, "0x403 SERVO_SET_SPEED", [("Set speed", self._do_servo_set_speed)]); r += 1
        self._cmd_row(cg, r, "0x404 SERVO_GET_SPEED", [("Read speed", self._do_servo_get_speed)]); r += 1
        self._cmd_row(cg, r, "0x405 SERVO_TORQUE", [
            ("On", lambda: self._do_servo_torque(True)),
            ("Off", lambda: self._do_servo_torque(False)),
            ("Calibrate", self._do_servo_calibrate),
            ("Query", self._do_servo_torque_query),
        ]); r += 1
        self._cmd_row(cg, r, "0x406 SERVO_TLIM", [("Set torque limit", self._do_servo_tlim)]); r += 1
        self._cmd_row(cg, r, "0x407 SERVO_MODE", [("Set mode", self._do_servo_mode)]); r += 1
        self._cmd_row(cg, r, "0x408 SERVO_PWM", [("Set PWM", self._do_servo_pwm)]); r += 1
        self._cmd_row(cg, r, "0x410 SERVO_GET_FB", [("Read feedback", self._do_servo_fb)]); r += 1
        self._cmd_row(cg, r, "0x412 SERVO_MEM_READ", [("Read register", self._do_servo_read)]); r += 1
        self._cmd_row(cg, r, "0x413 SERVO_MEM_WRITE", [("Write register", self._do_servo_write)]); r += 1
        self._cmd_row(cg, r, "0x414 SERVO_SET_ID", [("Set ID", self._do_servo_set_id)]); r += 1
        self._cmd_row(cg, r, "0x415 SERVO_RESET", [("Reset", self._do_servo_reset)]); r += 1
        cg.setColumnStretch(5, 1)

        fbox, self.log_servo = self._feedback_box("SERVO Feedback")
        return self._assemble_tab([pbox, cbox], fbox)

    # ==================================================================
    # Connection handling
    # ==================================================================

    def _refresh_ports(self):
        self.cmb_port.clear()
        ports = [p for p in TueUsbClient.list_ports() if p != "MOCK_PORT"]
        for port in ports:
            self.cmb_port.addItem(port)
        if not ports:
            self.cmb_port.addItem("MOCK_PORT")

    def _on_mock_toggled(self, checked: bool):
        self.client.mock = checked
        self._update_connection_ui()

    def _toggle_connection(self):
        if self.client.is_connected():
            self.client.disconnect()
            self.status.showMessage("Disconnected.")
        else:
            port = self.cmb_port.currentText()
            self.client.baudrate = self.cmb_baud.currentData()
            try:
                self.client.connect(port)
                self.status.showMessage(f"Connected to {port}.")
            except Exception as exc:  # noqa: BLE001
                QMessageBox.critical(self, "Connection error", f"Could not open {port}:\n{exc}")
        self._update_connection_ui()

    def _update_connection_ui(self):
        if self.client.is_connected():
            self.btn_connect.setText("Disconnect")
            self.lbl_conn.setText("MOCK" if self.client.mock else f"CONNECTED: {self.client.port}")
        else:
            self.btn_connect.setText("Connect")
            self.lbl_conn.setText("DISCONNECTED")

    # ==================================================================
    # Feedback log + async runner
    # ==================================================================

    def _run(self, log: QPlainTextEdit, label: str, func: Callable[[], object],
             fmt: Optional[Callable[[object], str]] = None):
        """Run func() in a worker; append '<label> -> <feedback>' to `log`."""
        if not self.client.is_connected():
            log.appendPlainText(f"{datetime.now():%H:%M:%S}  {label} -> not connected")
            return

        worker = AsyncWorker(func)

        def on_result(result):
            try:
                text = fmt(result) if fmt else str(result)
            except Exception as exc:  # noqa: BLE001
                text = f"<cannot format result: {exc}>"
            log.appendPlainText(f"{datetime.now():%H:%M:%S}  {label} -> {text}")

        def on_error(message: str):
            log.appendPlainText(f"{datetime.now():%H:%M:%S}  {label} -> ERROR: {message}")

        worker.result_ready.connect(on_result)
        worker.error_occurred.connect(on_error)
        worker.finished.connect(lambda: self._workers.remove(worker) if worker in self._workers else None)
        self._workers.append(worker)
        worker.start()

    # ==================================================================
    # System command handlers
    # ==================================================================

    @staticmethod
    def _parse_int(text: str) -> int:
        text = text.strip()
        return int(text, 16) if text.lower().startswith("0x") else int(text)

    def _do_ping(self):
        try:
            token = self._parse_int(self.txt_ping_token.text())
        except ValueError:
            self.log_sys.appendPlainText("0x100 SYS_PING -> ERROR: invalid token")
            return
        self._run(self.log_sys, "0x100 SYS_PING", lambda: self.client.ping(token),
                  lambda v: f"echo_token=0x{v:08X}")

    def _do_info(self):
        self._run(self.log_sys, "0x101 SYS_INFO", self.client.get_info,
                  lambda i: f"hw_id={i.hw_id}, hw_rev={i.hw_rev}, fw={i.version_string}, flags=0x{i.build_flags:08X}")

    def _do_uid(self):
        self._run(self.log_sys, "0x102/103 SYS_UID", self.client.get_uid, lambda u: u.hex_string)

    def _do_uptime(self):
        self._run(self.log_sys, "0x104 SYS_UPTIME", self.client.get_uptime_ms,
                  lambda ms: f"{ms} ms ({ms / 1000.0:.1f} s)")

    def _do_led(self, state: LedState):
        idx = self.spn_led_idx.value()
        self._run(self.log_sys, f"0x105 SYS_LED_CTRL[{state.name}]",
                  lambda: self.client.set_led(idx, state),
                  lambda v: f"led={idx}, state={'ON' if v else 'OFF'}")

    def _do_reset(self):
        try:
            magic = self._parse_int(self.txt_reset_magic.text())
        except ValueError:
            self.log_sys.appendPlainText("0x106 SYS_RESET -> ERROR: invalid magic")
            return
        if QMessageBox.question(self, "Confirm reset", "Reboot the MCU?") != QMessageBox.StandardButton.Yes:
            return
        self._run(self.log_sys, "0x106 SYS_RESET", lambda: self.client.reset_mcu(magic),
                  lambda _: "reset acknowledged (rebooting)")

    def _do_echo(self):
        data = self.txt_echo.text().encode("utf-8")[:8]
        self._run(self.log_sys, "0x107 SYS_ECHO", lambda: self.client.echo(data),
                  lambda b: f"{b.decode('utf-8', 'replace')} ({b.hex(' ')})")

    # ==================================================================
    # Stepper command handlers
    # ==================================================================

    def _do_step_info(self):
        self._run(self.log_step, "0x200 STEP_GET_INFO", self.client.step_get_config,
                  lambda c: f"XY={c.xy_steps_per_mm} steps/mm, Z={c.z_steps_per_mm} steps/mm, "
                            f"rate {c.min_rate_steps_s}-{c.max_rate_steps_s} step/s")

    def _do_step_status(self):
        self._run(self.log_step, "0x201 STEP_GET_STATUS", self.client.step_get_status,
                  lambda s: f"busy={s.is_busy}, enabled={s.is_enabled}, queue={s.queue_depth}")

    def _do_step_enable(self, enable):
        self._run(self.log_step, f"0x202 STEP_ENABLE[{enable}]",
                  lambda: self.client.step_enable(enable),
                  lambda v: f"enabled={v}")

    def _do_step_pos_xy(self):
        self._run(self.log_step, "0x203 STEP_GET_POS_XY", self.client.step_get_pos_xy,
                  lambda p: f"X={p[0]:.3f} mm, Y={p[1]:.3f} mm")

    def _do_step_pos_z(self):
        self._run(self.log_step, "0x204 STEP_GET_POS_Z", self.client.step_get_pos_z,
                  lambda z: f"Z={z:.3f} mm")

    def _do_step_tgt_xy(self):
        self._run(self.log_step, "0x205 STEP_GET_TGT_XY", self.client.step_get_tgt_xy,
                  lambda p: f"X={p[0]:.3f} mm, Y={p[1]:.3f} mm")

    def _do_step_tgt_z(self):
        self._run(self.log_step, "0x206 STEP_GET_TGT_Z", self.client.step_get_tgt_z,
                  lambda z: f"Z={z:.3f} mm")

    def _do_step_setpos_xy(self):
        x, y = self.spn_x.value(), self.spn_y.value()
        self._run(self.log_step, f"0x207 STEP_SETPOS_XY({x},{y})",
                  lambda: self.client.step_setpos_xy(x, y),
                  lambda p: f"X={p[0]:.3f} mm, Y={p[1]:.3f} mm")

    def _do_step_setpos_z(self):
        z = self.spn_z.value()
        self._run(self.log_step, f"0x208 STEP_SETPOS_Z({z})",
                  lambda: self.client.step_setpos_z(z),
                  lambda v: f"Z={v:.3f} mm")

    def _do_step_move_xyz(self):
        x, y, z = self.spn_x.value(), self.spn_y.value(), self.spn_z.value()
        feed, accel = self.spn_feed.value(), self.spn_accel.value()
        self._run(self.log_step, f"0x210/211 STEP_MOVE XYZ({x},{y},{z})",
                  lambda: self.client.step_move_xyz(x, y, z, feed, accel),
                  lambda p: f"committed X={p[0]:.3f} mm, Y={p[1]:.3f} mm")

    def _do_step_move_xy(self):
        x, y = self.spn_x.value(), self.spn_y.value()
        self._run(self.log_step, f"0x212 STEP_MOVE_XY({x},{y})",
                  lambda: self.client.step_move_xy(x, y),
                  lambda p: f"X={p[0]:.3f} mm, Y={p[1]:.3f} mm")

    def _do_step_move_z(self):
        z, feed, accel = self.spn_z.value(), self.spn_feed.value(), self.spn_accel.value()
        self._run(self.log_step, f"0x213 STEP_MOVE_Z({z})",
                  lambda: self.client.step_move_z(z, feed, accel),
                  lambda v: f"Z={v:.3f} mm")

    def _do_step_move_rel_xy(self):
        dx, dy = self.spn_x.value(), self.spn_y.value()
        self._run(self.log_step, f"0x214 STEP_MOVE_REL_XY({dx},{dy})",
                  lambda: self.client.step_move_rel_xy(dx, dy),
                  lambda p: f"dX={p[0]:.3f} mm, dY={p[1]:.3f} mm")

    def _do_step_set_speed(self):
        feed, accel = self.spn_feed.value(), self.spn_accel.value()
        self._run(self.log_step, f"0x215 STEP_SET_SPEED({feed},{accel})",
                  lambda: self.client.step_set_speed(feed, accel),
                  lambda v: f"feed={v[0]} mm/s, accel={v[1]} mm/s^2")

    def _do_step_jog_xy(self):
        vx, vy, accel = self.spn_jog_vx.value(), self.spn_jog_vy.value(), self.spn_accel.value()
        self._run(self.log_step, f"0x216 STEP_JOG_XY({vx},{vy})",
                  lambda: self.client.step_jog_xy(vx, vy, accel),
                  lambda v: f"vx={v[0]} mm/s, vy={v[1]} mm/s")

    def _do_step_jog_z(self):
        vz, accel = self.spn_jog_vz.value(), self.spn_accel.value()
        self._run(self.log_step, f"0x217 STEP_JOG_Z({vz})",
                  lambda: self.client.step_jog_z(vz, accel),
                  lambda v: f"vz={v} mm/s")

    def _do_step_stop(self):
        self._run(self.log_step, "0x218 STEP_STOP", self.client.step_stop, lambda v: f"busy={v}")

    def _do_step_abort(self):
        self._run(self.log_step, "0x219 STEP_ABORT", self.client.step_abort, lambda _: "aborted")

    def _do_step_wait_idle(self):
        timeout = self.spn_timeout.value()
        self._run(self.log_step, f"0x21A STEP_WAIT_IDLE({timeout})",
                  lambda: self.client.step_wait_idle(timeout),
                  lambda ms: f"elapsed={ms} ms")

    # ==================================================================
    # Raw stepper handlers
    # ==================================================================

    def _do_raw_read(self):
        motor = self.cmb_raw_motor.currentData()
        self._run(self.log_raw, f"0x280 STEP_RAW_GET_STEPS(motor {motor})",
                  lambda: self.client.step_raw_get_steps(motor),
                  lambda steps: f"steps={steps}")

    def _do_raw_move(self):
        motor = self.cmb_raw_motor.currentData()
        steps, feed = self.spn_raw_steps.value(), self.spn_raw_feed.value()
        self._run(self.log_raw, f"0x281 STEP_RAW_MOVE(motor {motor}, {steps})",
                  lambda: self.client.step_raw_move(motor, steps, feed),
                  lambda done: f"steps={done}")

    # ==================================================================
    # Servo command handlers
    # ==================================================================

    def _do_servo_ping(self):
        sid = self.spn_servo_id.value()
        self._run(self.log_servo, f"0x400 SERVO_PING({sid})", lambda: self.client.servo_ping(sid),
                  lambda ok: "FOUND" if ok else "NO RESPONSE")

    def _do_servo_set_pos(self):
        sid = self.spn_servo_id.value()
        pos = self.spn_servo_pos.value()
        speed = self.spn_servo_speed.value()
        accel = self.spn_servo_accel.value()
        self._run(self.log_servo, f"0x401 SERVO_SET_POS({sid}, pos={pos})",
                  lambda: self.client.servo_set_pos(sid, pos, speed, accel),
                  lambda v: f"accepted pos={v}")

    def _do_servo_get_pos(self):
        sid = self.spn_servo_id.value()
        self._run(self.log_servo, f"0x402 SERVO_GET_POS({sid})", lambda: self.client.servo_get_pos(sid),
                  lambda v: f"pos={v} ({v * 360.0 / 4096.0:.1f} deg)")

    def _do_servo_set_speed(self):
        sid = self.spn_servo_id.value()
        speed = self.spn_servo_speed.value()
        accel = self.spn_servo_accel.value()
        self._run(self.log_servo, f"0x403 SERVO_SET_SPEED({sid}, {speed})",
                  lambda: self.client.servo_set_speed(sid, speed, accel),
                  lambda v: f"accepted speed={v}")

    def _do_servo_get_speed(self):
        sid = self.spn_servo_id.value()
        self._run(self.log_servo, f"0x404 SERVO_GET_SPEED({sid})", lambda: self.client.servo_get_speed(sid),
                  lambda v: f"speed={v} steps/s")

    def _do_servo_torque(self, enable: bool):
        sid = self.spn_servo_id.value()
        self._run(self.log_servo, f"0x405 SERVO_TORQUE({sid}, {'ON' if enable else 'OFF'})",
                  lambda: self.client.servo_torque_enable(sid, enable),
                  lambda v: f"torque={'ON' if v else 'OFF'}")

    def _do_servo_calibrate(self):
        sid = self.spn_servo_id.value()
        self._run(self.log_servo, f"0x405 SERVO_TORQUE({sid}, CALIBRATE)",
                  lambda: self.client.servo_calibrate_2048(sid),
                  lambda _: "calibrated current position to 2048")

    def _do_servo_torque_query(self):
        sid = self.spn_servo_id.value()
        self._run(self.log_servo, f"0x405 SERVO_TORQUE({sid}, QUERY)",
                  lambda: self.client.servo_mem_read(sid, 0x28, 1),
                  lambda b: f"torque_switch=0x{b[0]:02X} ({'ON' if b[0] else 'OFF'})")

    def _do_servo_tlim(self):
        sid = self.spn_servo_id.value()
        limit = self.spn_servo_tlim.value()
        self._run(self.log_servo, f"0x406 SERVO_TLIM({sid}, {limit})",
                  lambda: self.client.servo_set_torque_limit(sid, limit),
                  lambda v: f"limit(0.1%)={v}")

    def _do_servo_mode(self):
        sid = self.spn_servo_id.value()
        mode = self.cmb_servo_mode.currentData()
        self._run(self.log_servo, f"0x407 SERVO_MODE({sid}, {mode})",
                  lambda: self.client.servo_set_mode(sid, mode),
                  lambda v: f"mode={v} ({ServoMode(v).name})")

    def _do_servo_pwm(self):
        sid = self.spn_servo_id.value()
        pwm = self.spn_servo_pwm.value()
        self._run(self.log_servo, f"0x408 SERVO_PWM({sid}, {pwm})",
                  lambda: self.client.servo_set_pwm(sid, pwm),
                  lambda v: f"pwm={v}")

    def _do_servo_fb(self):
        sid = self.spn_servo_id.value()

        def fmt(fb):
            d, h = fb.dynamics, fb.health
            alarms = ", ".join(h.alarms) if h.alarms else "none"
            return (f"pos={d.position} ({d.angle_deg:.1f} deg), speed={d.speed} step/s, "
                    f"load={d.load_percent:.1f}%, moving={d.is_moving}, "
                    f"{h.voltage_v:.1f} V, {h.temperature_c} C, {h.current_ma} mA, "
                    f"status=0x{h.status_mask:02X} [{alarms}]")

        self._run(self.log_servo, f"0x410 SERVO_GET_FB({sid})",
                  lambda: self.client.servo_get_feedback(sid), fmt)

    def _do_servo_read(self):
        sid = self.spn_servo_id.value()
        try:
            reg = self._parse_int(self.txt_servo_reg.text())
        except ValueError:
            self.log_servo.appendPlainText("0x412 SERVO_MEM_READ -> ERROR: invalid register address")
            return
        length = self.spn_servo_len.value()
        self._run(self.log_servo, f"0x412 SERVO_MEM_READ({sid}, reg=0x{reg:02X}, len={length})",
                  lambda: self.client.servo_mem_read(sid, reg, length),
                  lambda data: f"reg 0x{reg:02X} = {data.hex(' ')} (int={int.from_bytes(data, 'little')})")

    def _do_servo_write(self):
        sid = self.spn_servo_id.value()
        try:
            reg = self._parse_int(self.txt_servo_reg.text())
            data = bytes.fromhex(self.txt_servo_wdata.text().replace(" ", "").replace("0x", ""))
        except ValueError:
            self.log_servo.appendPlainText("0x413 SERVO_MEM_WRITE -> ERROR: invalid register or hex data")
            return
        if not data:
            self.log_servo.appendPlainText("0x413 SERVO_MEM_WRITE -> ERROR: no data bytes")
            return
        self._run(self.log_servo, f"0x413 SERVO_MEM_WRITE({sid}, reg=0x{reg:02X}, {data.hex(' ')})",
                  lambda: self.client.servo_mem_write(sid, reg, data),
                  lambda status: f"status={status}")

    def _do_servo_set_id(self):
        sid = self.spn_servo_id.value()
        new_id = self.spn_servo_new_id.value()
        self._run(self.log_servo, f"0x414 SERVO_SET_ID({sid} -> {new_id})",
                  lambda: self.client.servo_set_id(sid, new_id),
                  lambda status: f"new_id={new_id}, status={status}")

    def _do_servo_reset(self):
        sid = self.spn_servo_id.value()
        self._run(self.log_servo, f"0x415 SERVO_RESET({sid})", lambda: self.client.servo_reset(sid),
                  lambda status: f"status={status}")


def run_app() -> int:
    app = QApplication.instance() or QApplication(sys.argv)
    window = TueUsbApp()
    window.show()
    return app.exec()


if __name__ == "__main__":
    sys.exit(run_app())
