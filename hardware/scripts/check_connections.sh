#!/usr/bin/env bash
# check_connections.sh
#
# Verifies that the expected serial devices are present and lists all
# currently connected USB-serial adapters.
#
# Usage:
#   bash hardware/scripts/check_connections.sh

LEADER_PORT="/dev/serial/by-id/usb-1a86_USB_Single_Serial_5AAF218344-if00"
FOLLOWER_PORT="/dev/serial/by-id/usb-1a86_USB_Single_Serial_5AAF219983-if00"

echo "=== SO-100 Hardware Connection Check ==="
echo ""

check_device() {
    local PORT="$1"
    local NAME="$2"
    if [ -e "$PORT" ]; then
        REAL=$(realpath "$PORT" 2>/dev/null)
        echo "[✓] $NAME found:  $PORT  →  $REAL"
    else
        echo "[✗] $NAME NOT found:  $PORT"
    fi
}

check_device "$LEADER_PORT"   "Leader  arm"
check_device "$FOLLOWER_PORT" "Follower arm"

echo ""
echo "--- All /dev/serial/by-id/ devices ---"
if [ -d /dev/serial/by-id ]; then
    ls -1 /dev/serial/by-id/ 2>/dev/null || echo "  (none)"
else
    echo "  /dev/serial/by-id/ directory not found"
fi

echo ""
echo "--- USB device overview (lsusb) ---"
lsusb 2>/dev/null || echo "  lsusb not available"
