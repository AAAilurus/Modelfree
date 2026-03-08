#!/usr/bin/env bash
# setup_serial.sh
#
# Grant read/write permissions to the SO-100 / SO-101 serial devices.
# Run this once after plugging in the USB adapters (or add a udev rule
# for a permanent fix).
#
# Usage:
#   bash hardware/scripts/setup_serial.sh

LEADER_PORT="/dev/serial/by-id/usb-1a86_USB_Single_Serial_5AAF218344-if00"
FOLLOWER_PORT="/dev/serial/by-id/usb-1a86_USB_Single_Serial_5AAF219983-if00"

set_permissions() {
    local PORT="$1"
    local NAME="$2"

    if [ -e "$PORT" ]; then
        sudo chmod 666 "$PORT"
        # Also chmod the underlying tty device (the by-id path is a symlink)
        REAL=$(realpath "$PORT" 2>/dev/null)
        if [ -n "$REAL" ]; then
            sudo chmod 666 "$REAL"
            echo "[OK] $NAME  →  $REAL  (permissions set)"
        fi
    else
        echo "[WARN] $NAME device not found: $PORT"
        echo "       Check: ls -l /dev/serial/by-id/"
    fi
}

echo "=== SO-100 Hardware Serial Setup ==="
set_permissions "$LEADER_PORT"   "Leader  arm"
set_permissions "$FOLLOWER_PORT" "Follower arm"

echo ""
echo "Tip: to make this permanent, create a udev rule:"
echo "  echo 'SUBSYSTEM==\"tty\", ATTRS{idVendor}==\"1a86\", MODE=\"0666\"' | \\"
echo "       sudo tee /etc/udev/rules.d/99-so100-serial.rules"
echo "  sudo udevadm control --reload-rules && sudo udevadm trigger"
