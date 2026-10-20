#!/usr/bin/env bash
# Build and flash the delivery-box firmware to the ESP32-S3.
#
#   ./flash.sh              build + flash (port found automatically)
#   ./flash.sh -m           ...then open the serial monitor (Ctrl+] to quit)
#   ./flash.sh -e           erase the whole flash first, then build + flash
#   ./flash.sh --check      check ESP-IDF, target and port, and build; do not flash
#   PORT=/dev/ttyACM1 ./flash.sh    use a specific serial port
#
# Options can be combined: ./flash.sh -e -m
set -euo pipefail

IDF_EXPORT="${IDF_EXPORT:-$HOME/.espressif/tools/activate_idf_v5.2.7.sh}"   # EIM install
TARGET="esp32s3"

MONITOR=0
ERASE=0
CHECK_ONLY=0
for arg in "$@"; do
    case "$arg" in
        -m|--monitor) MONITOR=1 ;;
        -e|--erase)   ERASE=1 ;;
        --check)      CHECK_ONLY=1 ;;
        -h|--help)    sed -n '2,10p' "$0"; exit 0 ;;
        *) echo "Unknown option: $arg (see ./flash.sh --help)" >&2; exit 1 ;;
    esac
done

die() { echo "ERROR: $*" >&2; exit 1; }

cd "$(dirname "$(readlink -f "$0")")"

# ---- ESP-IDF -----------------------------------------------------------------
[[ -x "$IDF_EXPORT" ]] || die "ESP-IDF not found at $IDF_EXPORT (set IDF_EXPORT=...)"

# Do NOT source the activation script here: it checks the name of the
# running program and calls `exit 1` when that is not "bash"/"sh", which
# would silently end this script. Its -e mode prints the variables instead.
while IFS='=' read -r key value; do
    case "$key" in
        PATH)           export PATH="$value:$PATH" ;;
        SYSTEM_PATH|"") ;;
        *)              export "$key=$value" ;;
    esac
done < <("$IDF_EXPORT" -e)

[[ -n "${IDF_PATH:-}" && -n "${IDF_PYTHON_ENV_PATH:-}" ]] \
    || die "ESP-IDF did not activate (no IDF_PATH from $IDF_EXPORT -e)"
idf() { "$IDF_PYTHON_ENV_PATH/bin/python" "$IDF_PATH/tools/idf.py" "$@"; }
echo "ESP-IDF: $(idf --version 2>/dev/null)"

# ---- target ------------------------------------------------------------------
# A build folder or sdkconfig made for another chip would flash the wrong image.
if [[ -f sdkconfig ]] && ! grep -q "^CONFIG_IDF_TARGET=\"$TARGET\"" sdkconfig; then
    echo "sdkconfig is not for $TARGET: setting the target (this rebuilds everything)"
    idf set-target "$TARGET"
fi

# ---- serial port ---------------------------------------------------------------
# The board's native USB port is /dev/ttyACM*; a USB-UART adapter is /dev/ttyUSB*.
if [[ -z "${PORT:-}" ]]; then
    ports=( /dev/ttyACM* /dev/ttyUSB* )
    found=()
    for p in "${ports[@]}"; do [[ -e "$p" ]] && found+=( "$p" ); done
    [[ ${#found[@]} -gt 0 ]] || die "no ESP32 serial port found. Is the board plugged in over USB?"
    if [[ ${#found[@]} -gt 1 ]]; then
        die "several serial ports: ${found[*]} - pick one, e.g. PORT=${found[0]} ./flash.sh"
    fi
    PORT="${found[0]}"
fi
[[ -e "$PORT" ]] || die "serial port $PORT not found. Is the board plugged in?"
[[ -r "$PORT" && -w "$PORT" ]] || die "no permission for $PORT: run 'sudo usermod -aG dialout $USER', then log out and in"

# Another program (a monitor, a terminal) holding the port makes flashing fail.
if command -v fuser >/dev/null 2>&1 && fuser "$PORT" >/dev/null 2>&1; then
    die "$PORT is in use by another program (close idf.py monitor / other terminals): $(fuser "$PORT" 2>&1)"
fi
echo "Port:    $PORT"

if [[ $CHECK_ONLY -eq 1 ]]; then
    idf build
    echo "Check OK: it builds and $PORT is ready (nothing was flashed)."
    exit 0
fi

# ---- build + flash ---------------------------------------------------------------
if [[ $ERASE -eq 1 ]]; then
    idf -p "$PORT" erase-flash
fi

steps=( build flash )
[[ $MONITOR -eq 1 ]] && steps+=( monitor )

if ! idf -p "$PORT" "${steps[@]}"; then
    cat >&2 <<EOF

Flashing failed. If the board did not enter download mode by itself:
  1. hold the BOOT button, press and release RESET, release BOOT
  2. run ./flash.sh again
EOF
    exit 1
fi
