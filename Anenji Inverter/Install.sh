#!/bin/bash
# =============================================================================
#  Install.sh — ANENJI ANJ-3KW-24V-LV-WIFI standalone Venus OS driver installer
# =============================================================================

set -euo pipefail

RED='\033[0;31m'; GREEN='\033[0;32m'; YELLOW='\033[1;33m'; NC='\033[0m'
info()  { echo -e "${GREEN}[INFO]${NC}  $*"; }
warn()  { echo -e "${YELLOW}[WARN]${NC}  $*"; }
error() { echo -e "${RED}[ERROR]${NC} $*" >&2; exit 1; }

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
INSTALL_DIR="/data/etc/dbus-anenji"
SERVICE_DIR="/service/dbus-anenji"
LOG_DIR="/var/log/dbus-anenji"
DRIVER_SRC="${SCRIPT_DIR}/dbus-anenji.py"
README_SRC="${SCRIPT_DIR}/README.md"
RC_LOCAL="/data/rc.local"
VENUS_VERSION_FILE="/opt/victronenergy/version"
VELIB_NATIVE="/opt/victronenergy/dbus-systemcalc-py/ext/velib_python"
VELIB_FALLBACK="/opt/victronenergy/dbus-battery/ext/velib_python"
VELIB_LEGACY="/data/apps/dbus-serialbattery/ext/velib_python"

info "Checking environment …"
[[ $EUID -eq 0 ]] || error "This script must be run as root."
[[ -f "$DRIVER_SRC" ]] || error "dbus-anenji.py not found in ${SCRIPT_DIR}."
[[ -f "$README_SRC" ]] || warn "README.md not found next to installer — continuing anyway."

if [[ -f "$VENUS_VERSION_FILE" ]]; then
    VENUS_VER=$(tr -d '\n\r' < "$VENUS_VERSION_FILE")
    info "Detected Venus OS version: ${VENUS_VER}"
    MAJOR=$(echo "$VENUS_VER" | tr -d 'v' | cut -d. -f1)
    [[ -n "$MAJOR" && "$MAJOR" -ge 3 ]] || error "Venus OS v3.00 or newer is required. Found: ${VENUS_VER}"
else
    warn "Cannot find ${VENUS_VERSION_FILE} — skipping version check."
fi

if [[ ! -d "$VELIB_NATIVE" && ! -d "$VELIB_FALLBACK" && ! -d "$VELIB_LEGACY" ]]; then
    warn "velib_python was not found in the standard Venus OS locations."
    warn "The driver may fail to start until velib_python is available."
fi

info "Creating installation directory: ${INSTALL_DIR}"
mkdir -p "$INSTALL_DIR"

info "Copying driver payload …"
cp -f "$DRIVER_SRC" "$INSTALL_DIR/dbus-anenji.py"
chmod 755 "$INSTALL_DIR/dbus-anenji.py"
cp -f "${BASH_SOURCE[0]}" "$INSTALL_DIR/Install.sh" 2>/dev/null || true
[[ -f "$README_SRC" ]] && cp -f "$README_SRC" "$INSTALL_DIR/README.md"

info "Writing enable.sh …"
cat > "$INSTALL_DIR/enable.sh" <<'ENABLE_SH'
#!/bin/bash
set -euo pipefail

INSTALL_DIR="/data/etc/dbus-anenji"
SERVICE_DIR="/service/dbus-anenji"
LOG_DIR="/var/log/dbus-anenji"

mkdir -p "$LOG_DIR"
mkdir -p "$SERVICE_DIR/log"

cat > "$SERVICE_DIR/log/run" <<'LOG_RUN'
#!/bin/sh
exec multilog t s500000 n4 /var/log/dbus-anenji
LOG_RUN
chmod 755 "$SERVICE_DIR/log/run"

cat > "$SERVICE_DIR/run" <<'SVC_RUN'
#!/bin/sh
trap 'kill -TERM $PID' TERM INT
exec 2>&1
python3 /data/etc/dbus-anenji/dbus-anenji.py &
PID=$!
wait $PID
exit $?
SVC_RUN
chmod 755 "$SERVICE_DIR/run"

echo "[enable.sh] Service directory (re-)created at ${SERVICE_DIR}"
ENABLE_SH
chmod 755 "$INSTALL_DIR/enable.sh"

info "Writing uninstall.sh …"
cat > "$INSTALL_DIR/uninstall.sh" <<'UNINSTALL_SH'
#!/bin/bash
set -euo pipefail

SERVICE_DIR="/service/dbus-anenji"
INSTALL_DIR="/data/etc/dbus-anenji"
RC_LOCAL="/data/rc.local"

svc -d "$SERVICE_DIR" 2>/dev/null || true
sleep 1
rm -rf "$SERVICE_DIR"

if command -v pgrep >/dev/null 2>&1; then
    for pid in $(pgrep -f 'dbus-anenji.py' || true); do
        kill "$pid" 2>/dev/null || true
    done
fi

sed -i '/dbus-anenji/d' "$RC_LOCAL" 2>/dev/null || true
rm -rf "$INSTALL_DIR"

echo "ANENJI driver uninstall complete."
UNINSTALL_SH
chmod 755 "$INSTALL_DIR/uninstall.sh"

info "Running enable.sh …"
bash "$INSTALL_DIR/enable.sh"

info "Hooking enable.sh into ${RC_LOCAL} …"
if [[ ! -f "$RC_LOCAL" ]]; then
    echo '#!/bin/bash' > "$RC_LOCAL"
    chmod 755 "$RC_LOCAL"
fi
HOOK_LINE='bash /data/etc/dbus-anenji/enable.sh >> /data/etc/dbus-anenji/startup.log 2>&1'
if grep -qxF "$HOOK_LINE" "$RC_LOCAL"; then
    info "rc.local hook already present — skipping."
else
    echo "$HOOK_LINE" >> "$RC_LOCAL"
    info "rc.local hook added."
fi

info "Starting dbus-anenji service …"
sleep 2
svc -u "$SERVICE_DIR" 2>/dev/null || true

STATUS="unknown"
if command -v svstat >/dev/null 2>&1; then
    STATUS=$(svstat "$SERVICE_DIR" 2>/dev/null || true)
elif command -v sv >/dev/null 2>&1; then
    STATUS=$(sv status "$SERVICE_DIR" 2>/dev/null || true)
fi

echo
echo -e "${GREEN}═══════════════════════════════════════════════════════════${NC}"
echo -e "${GREEN}  ANENJI Venus OS driver installed successfully!          ${NC}"
echo -e "${GREEN}═══════════════════════════════════════════════════════════${NC}"
echo "Install directory : $INSTALL_DIR"
echo "Service directory : $SERVICE_DIR"
echo "Log directory     : $LOG_DIR"
echo "Service status    : $STATUS"
echo "Read the docs     : $INSTALL_DIR/README.md"
