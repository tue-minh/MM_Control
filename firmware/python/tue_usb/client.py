"""
TUE USB Serial Client.

Provides a thread-safe, high-level client interface communicating over USB CDC using pyserial.
"""

import time
import threading
from typing import Any, Dict, List, Optional, Tuple, Union

try:
    import serial
    import serial.tools.list_ports
except ImportError:
    serial = None  # type: ignore

from .defs import CommandId, LedMode, StatusCode
from .exceptions import (
    TueConnectionError,
    TueError,
    TueProtocolError,
    TueTimeoutError,
)
from .protocol import (
    TueFrame,
    TueFrameParser,
    decode_info_response,
    decode_led_response,
    encode_frame,
    encode_led_cmd,
)


class TueClient:
    """Thread-safe USB CDC communication client for TUE motion controllers."""

    def __init__(
        self,
        port: str,
        baudrate: int = 115200,
        timeout: float = 1.0,
    ) -> None:
        """Initialize the TUE USB Client.

        Args:
            port: Serial device path (e.g. "COM3" on Windows, "/dev/ttyACM0" on Linux).
            baudrate: Baudrate (ignored by virtual USB CDC, but kept for pyserial compatibility).
            timeout: Default timeout in seconds for operations.
        """
        if serial is None:
            raise ImportError(
                "pyserial is required to use TueClient. Install with: pip install pyserial"
            )

        self._port_name: str = port
        self._baudrate: int = baudrate
        self._default_timeout: float = timeout
        self._ser: Optional[serial.Serial] = None
        self._seq: int = 0
        self._lock = threading.Lock()
        self._parser = TueFrameParser()

    @staticmethod
    def list_ports() -> List[str]:
        """List all available serial port device names."""
        if serial is None:
            return []
        ports = serial.tools.list_ports.comports()
        return [p.device for p in ports]

    def open(self) -> None:
        """Open the serial port connection."""
        if self._ser is not None and self._ser.is_open:
            return

        try:
            self._ser = serial.Serial(
                port=self._port_name,
                baudrate=self._baudrate,
                timeout=0.05,  # Non-blocking poll chunks
                write_timeout=self._default_timeout,
            )
            self._parser.reset()
            # Flush any stale data
            self._ser.reset_input_buffer()
            self._ser.reset_output_buffer()
        except Exception as e:
            raise TueConnectionError(
                f"Failed to open serial port '{self._port_name}': {e}"
            ) from e

    def close(self) -> None:
        """Close the serial port connection."""
        if self._ser is not None:
            try:
                self._ser.close()
            except Exception:
                pass
            self._ser = None

    @property
    def is_open(self) -> bool:
        """Check if serial connection is open."""
        return self._ser is not None and self._ser.is_open

    def __enter__(self) -> "TueClient":
        self.open()
        return self

    def __exit__(self, exc_type: Any, exc_val: Any, exc_tb: Any) -> None:
        self.close()

    def _next_seq(self) -> int:
        seq = self._seq
        self._seq = (self._seq + 1) & 0xFF
        return seq

    def send_command(
        self,
        cmd: Union[int, CommandId],
        payload: bytes = b"",
        timeout: Optional[float] = None,
    ) -> TueFrame:
        """Send a request frame and synchronously await the corresponding response.

        Args:
            cmd: Command ID.
            payload: Payload bytes.
            timeout: Timeout in seconds (defaults to client timeout).

        Returns:
            The received and validated TueFrame response.

        Raises:
            TueTimeoutError: If response is not received within timeout.
            TueProtocolError: If MCU returns a non-OK status.
            TueConnectionError: If connection is broken or closed.
        """
        if not self.is_open:
            self.open()

        cmd_val = int(cmd) & 0xFF
        deadline = time.time() + (timeout if timeout is not None else self._default_timeout)

        with self._lock:
            assert self._ser is not None
            seq = self._next_seq()
            raw_frame = encode_frame(seq=seq, cmd=cmd_val, status=StatusCode.OK, payload=payload)

            # Flush input buffer before sending request
            self._ser.reset_input_buffer()
            self._parser.reset()

            try:
                self._ser.write(raw_frame)
                self._ser.flush()
            except Exception as e:
                raise TueConnectionError(f"Serial write error: {e}") from e

            # Read response until matching SEQ and CMD are found
            while time.time() < deadline:
                chunk = self._ser.read(self._ser.in_waiting or 1)
                if chunk:
                    frames = self._parser.feed(chunk)
                    for frame in frames:
                        if frame.seq == seq and frame.cmd == cmd_val:
                            if not frame.is_ok:
                                raise TueProtocolError(frame.status)
                            return frame
                else:
                    time.sleep(0.002)

            raise TueTimeoutError(
                f"Timed out waiting for response to CMD 0x{cmd_val:02X} (seq={seq})"
            )

    # ==========================================================================
    # High-Level Subsystem APIs
    # ==========================================================================

    def ping(self, data: bytes = b"TUE_PING") -> bytes:
        """Send a ping echo request.

        Args:
            data: Arbitrary bytes to echo.

        Returns:
            Echoed payload bytes from MCU.
        """
        resp = self.send_command(CommandId.SYS_PING, payload=data)
        return resp.payload

    def get_info(self) -> Dict[str, Any]:
        """Query board ID, firmware version, and STM32 Unique ID.

        Returns:
            Dictionary with board info fields.
        """
        resp = self.send_command(CommandId.SYS_GET_INFO)
        return decode_info_response(resp.payload)

    def set_led(self, led_index: int = 0, state: bool = True) -> bool:
        """Set onboard LED state.

        Args:
            led_index: Index of LED (0 = LED_MCU).
            state: True for ON, False for OFF.

        Returns:
            Current pin state (True = ON, False = OFF).
        """
        mode = LedMode.ON if state else LedMode.OFF
        payload = encode_led_cmd(led_index, mode)
        resp = self.send_command(CommandId.SYS_LED_CTRL, payload=payload)
        _, current_state = decode_led_response(resp.payload)
        return bool(current_state)

    def toggle_led(self, led_index: int = 0) -> bool:
        """Toggle onboard LED state.

        Args:
            led_index: Index of LED (0 = LED_MCU).

        Returns:
            Current pin state after toggle (True = ON, False = OFF).
        """
        payload = encode_led_cmd(led_index, LedMode.TOGGLE)
        resp = self.send_command(CommandId.SYS_LED_CTRL, payload=payload)
        _, current_state = decode_led_response(resp.payload)
        return bool(current_state)

    def get_led(self, led_index: int = 0) -> bool:
        """Query onboard LED state without altering it.

        Args:
            led_index: Index of LED (0 = LED_MCU).

        Returns:
            Current pin state (True = ON, False = OFF).
        """
        payload = encode_led_cmd(led_index, LedMode.QUERY)
        resp = self.send_command(CommandId.SYS_LED_CTRL, payload=payload)
        _, current_state = decode_led_response(resp.payload)
        return bool(current_state)
