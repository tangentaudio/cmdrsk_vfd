"""VFD Monitor — Probe Basic sidebar widget for Commander SK status.

Creates a small HAL component ('vfd-monitor') with input pins that mirror
the cmdrsk_vfd driver outputs.  Monitors pin state changes and spawns
popup dialogs for drawbar and fault events.

All files live in the machine config directory — no Probe Basic core
modifications are required.
"""

import os
import importlib.util

from qtpy import uic
from qtpy.QtCore import Qt, QTimer
from qtpy.QtWidgets import QWidget, QDialog, QVBoxLayout, QLabel, QPushButton

from qtpyvcp.plugins import getPlugin
from qtpyvcp.utilities import logger
from qtpyvcp.hal import QComponent

# Load trip_codes from the same directory (relative imports don't work
# under Probe Basic's user tab loader)
_tc_path = os.path.join(os.path.dirname(__file__), "trip_codes.py")
_tc_spec = importlib.util.spec_from_file_location("trip_codes", _tc_path)
trip_codes = importlib.util.module_from_spec(_tc_spec)
_tc_spec.loader.exec_module(trip_codes)

LOG = logger.getLogger(__name__)

# Style sheets for popup dialogs
DRAWBAR_STYLE = """
QDialog {
    background-color: #3d3520;
    border: 2px solid #d4a017;
    border-radius: 8px;
}
QLabel {
    color: #ffd54f;
    font-size: 14px;
}
QLabel#popupTitle {
    font-size: 18px;
    font-weight: bold;
}
"""

FAULT_STYLE = """
QDialog {
    background-color: #3d1a1a;
    border: 2px solid #e53935;
    border-radius: 8px;
}
QLabel {
    color: #ff8a80;
    font-size: 14px;
}
QLabel#popupTitle {
    font-size: 18px;
    font-weight: bold;
    color: #ff5252;
}
QPushButton {
    background-color: #b71c1c;
    color: white;
    border: 1px solid #e53935;
    border-radius: 4px;
    padding: 8px 16px;
    font-size: 14px;
    font-weight: bold;
}
QPushButton:hover {
    background-color: #d32f2f;
}
QPushButton:pressed {
    background-color: #c62828;
}
"""

# Sidebar label styles
STYLE_OK = "color: #4caf50; font-weight: bold;"
STYLE_WARN = "color: #ffc107; font-weight: bold;"
STYLE_FAULT = "color: #f44336; font-weight: bold;"
STYLE_IDLE = "color: #9e9e9e;"


class DrawbarPopup(QDialog):
    """Popup shown while the power drawbar is active.

    Uses X11BypassWindowManagerHint to avoid triggering the WM
    (which would raise the taskbar in fullscreen mode).
    """

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setWindowFlags(
            Qt.FramelessWindowHint
            | Qt.X11BypassWindowManagerHint
            | Qt.WindowDoesNotAcceptFocus
        )
        self.setStyleSheet(DRAWBAR_STYLE)
        self.setFixedSize(340, 120)

        layout = QVBoxLayout(self)
        layout.setContentsMargins(16, 16, 16, 16)

        title = QLabel("\u26a0  Power Drawbar Active")
        title.setObjectName("popupTitle")
        title.setAlignment(Qt.AlignCenter)
        layout.addWidget(title)

        subtitle = QLabel("Tool release in progress \u2014 spindle locked out")
        subtitle.setAlignment(Qt.AlignCenter)
        subtitle.setWordWrap(True)
        layout.addWidget(subtitle)


class FaultPopup(QDialog):
    """Popup shown when the VFD is in fault state.

    Uses X11BypassWindowManagerHint to stay invisible to the WM.
    """

    def __init__(self, trip_code=0, parent=None):
        super().__init__(parent)
        self.setWindowFlags(
            Qt.FramelessWindowHint
            | Qt.X11BypassWindowManagerHint
            | Qt.WindowDoesNotAcceptFocus
        )
        self.setStyleSheet(FAULT_STYLE)
        self.setFixedSize(400, 200)

        layout = QVBoxLayout(self)
        layout.setContentsMargins(16, 16, 16, 16)

        name = trip_codes.trip_name(trip_code)
        desc = trip_codes.trip_description(trip_code)

        title = QLabel("\U0001f6d1  VFD FAULT: {}".format(name))
        title.setObjectName("popupTitle")
        title.setAlignment(Qt.AlignCenter)
        layout.addWidget(title)

        desc_label = QLabel(desc)
        desc_label.setAlignment(Qt.AlignCenter)
        desc_label.setWordWrap(True)
        layout.addWidget(desc_label)

        code_label = QLabel("Trip code: {}".format(trip_code))
        code_label.setAlignment(Qt.AlignCenter)
        layout.addWidget(code_label)

        self.reset_btn = QPushButton("Reset VFD")
        layout.addWidget(self.reset_btn)

    def update_trip(self, trip_code):
        """Update displayed trip code without recreating the dialog."""
        name = trip_codes.trip_name(trip_code)
        desc = trip_codes.trip_description(trip_code)
        labels = self.findChildren(QLabel)
        if len(labels) >= 3:
            labels[0].setText("\U0001f6d1  VFD FAULT: {}".format(name))
            labels[1].setText(desc)
            labels[2].setText("Trip code: {}".format(trip_code))


class UserTab(QWidget):
    """Probe Basic sidebar user tab for VFD monitoring.

    Creates a HAL component 'vfd-monitor' with pins:
        IN:  fault (bit), trip-code (s32), drawbar-active (bit), drive-ok (bit)
        OUT: fault-reset (bit)
    """

    def __init__(self, parent=None):
        super().__init__(parent)

        # Load the .ui file
        ui_file = os.path.join(
            os.path.dirname(__file__), "vfd_monitor.ui"
        )
        uic.loadUi(ui_file, self)

        # Popup dialog references
        self._drawbar_popup = None
        self._fault_popup = None

        # Create HAL component
        try:
            self._hal = QComponent("vfd-monitor")
            self._hal.addPin("fault", "bit", "in")
            self._hal.addPin("trip-code", "s32", "in")
            self._hal.addPin("drawbar-active", "bit", "in")
            self._hal.addPin("drive-ok", "bit", "in")
            self._hal.addPin("fault-reset", "bit", "out")
            self._hal.ready()

            # Connect value-changed signals
            self._hal.addListener("fault", self._on_fault_changed)
            self._hal.addListener("drawbar-active", self._on_drawbar_changed)
            self._hal.addListener("drive-ok", self._on_drive_ok_changed)
            self._hal.addListener("trip-code", self._on_trip_code_changed)

            LOG.info("VFD Monitor: HAL component ready")
        except Exception as e:
            LOG.error("VFD Monitor: failed to create HAL component: %s", e)
            self._hal = None

        # Connect sidebar reset button
        self.resetButton.clicked.connect(self._on_reset_clicked)
        self.resetButton.setEnabled(False)

        # Initial state
        self.driveStatus.setText("\u2014")
        self.driveStatus.setStyleSheet(STYLE_IDLE)
        self.drawbarStatus.setText("Idle")
        self.drawbarStatus.setStyleSheet(STYLE_IDLE)
        self.faultStatus.setText("None")
        self.faultStatus.setStyleSheet(STYLE_IDLE)

    # ------------------------------------------------------------------
    # HAL pin change handlers
    # ------------------------------------------------------------------

    def _on_drive_ok_changed(self, value):
        if value:
            self.driveStatus.setText("OK")
            self.driveStatus.setStyleSheet(STYLE_OK)
        else:
            self.driveStatus.setText("NOT OK")
            self.driveStatus.setStyleSheet(STYLE_FAULT)

    def _on_drawbar_changed(self, active):
        if active:
            # Only show popup if VFD is actually communicating;
            # when unpowered, hardware-enable defaults to FALSE which
            # would falsely indicate drawbar active.
            drive_ok = (self._hal and self._hal.getPin("drive-ok").value)
            self.drawbarStatus.setText("ACTIVE")
            self.drawbarStatus.setStyleSheet(STYLE_WARN)
            if drive_ok:
                self._show_drawbar_popup()
        else:
            self.drawbarStatus.setText("Idle")
            self.drawbarStatus.setStyleSheet(STYLE_IDLE)
            self._dismiss_drawbar_popup()

    def _on_fault_changed(self, faulted):
        if faulted:
            self.faultStatus.setStyleSheet(STYLE_FAULT)
            self.resetButton.setEnabled(True)
            # Trip code may not be updated yet (QPin polls at 100ms).
            # Show popup now with whatever we have; _on_trip_code_changed
            # will update it when the real code arrives.
            trip_code = 0
            if self._hal:
                trip_code = self._hal.getPin("trip-code").value
            self._update_fault_display(trip_code)
        else:
            self.faultStatus.setText("None")
            self.faultStatus.setStyleSheet(STYLE_IDLE)
            self.resetButton.setEnabled(False)
            self._dismiss_fault_popup()
            LOG.info("VFD fault cleared")

    def _on_trip_code_changed(self, code):
        """Called when trip-code pin value changes (may arrive after fault)."""
        if self._hal and self._hal.getPin("fault").value and code != 0:
            self._update_fault_display(code)

    def _update_fault_display(self, trip_code):
        """Update sidebar and popup with the current trip code."""
        name = trip_codes.trip_name(trip_code)
        desc = trip_codes.trip_description(trip_code)
        self.faultStatus.setText(name)
        if trip_code != 0:
            self._show_fault_popup(trip_code)
            LOG.error("VFD FAULT: %s \u2014 %s (code %d)", name, desc, trip_code)
        else:
            # Code 0 means we're still waiting for the real code
            self._show_fault_popup(trip_code)

    # ------------------------------------------------------------------
    # Popup management
    # ------------------------------------------------------------------

    def _show_drawbar_popup(self):
        if self._drawbar_popup is not None:
            return  # already showing
        self._drawbar_popup = DrawbarPopup(self.window())
        self._center_popup(self._drawbar_popup)
        self._drawbar_popup.show()

    def _dismiss_drawbar_popup(self):
        if self._drawbar_popup is not None:
            self._drawbar_popup.close()
            self._drawbar_popup.deleteLater()
            self._drawbar_popup = None

    def _show_fault_popup(self, trip_code):
        if self._fault_popup is not None:
            self._fault_popup.update_trip(trip_code)
            return  # already showing, just update
        self._fault_popup = FaultPopup(trip_code, self.window())
        self._fault_popup.reset_btn.clicked.connect(self._on_reset_clicked)
        self._center_popup(self._fault_popup)
        self._fault_popup.show()

    def _dismiss_fault_popup(self):
        if self._fault_popup is not None:
            self._fault_popup.close()
            self._fault_popup.deleteLater()
            self._fault_popup = None

    def _center_popup(self, popup):
        """Center a popup on the screen over the main window."""
        parent = self.window()
        if parent:
            geo = parent.geometry()
            x = geo.x() + (geo.width() - popup.width()) // 2
            y = geo.y() + (geo.height() - popup.height()) // 2
            popup.move(x, y)

    # ------------------------------------------------------------------
    # Reset button
    # ------------------------------------------------------------------

    def _on_reset_clicked(self):
        """Send a rising edge on the fault-reset HAL pin."""
        if not self._hal:
            return
        LOG.info("VFD Monitor: sending fault reset")
        pin = self._hal.getPin("fault-reset")
        pin.value = True
        # Deassert after 500ms
        QTimer.singleShot(500, lambda: setattr(pin, 'value', False))
