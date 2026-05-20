#!/usr/bin/env bash
# Build and flash custom LED matrix firmware with DisplayEQ support.
# Run from anywhere; it will place the inputmodule-rs clone next to this script.
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_DIR="$SCRIPT_DIR/inputmodule-rs"
REPO_URL="https://github.com/FrameworkComputer/inputmodule-rs.git"
TARGET="thumbv6m-none-eabi"
BINARY="ledmatrix"

# ---------------------------------------------------------------------------
# 0. System dependencies (libudev required by elf2uf2-rs)
# ---------------------------------------------------------------------------
echo "==> Checking system dependencies..."
if command -v dnf &>/dev/null; then
    if ! rpm -q systemd-devel &>/dev/null; then
        echo "  Installing systemd-devel (provides libudev)..."
        sudo dnf install -y systemd-devel
    fi
elif command -v apt-get &>/dev/null; then
    if ! dpkg -s libudev-dev &>/dev/null 2>&1; then
        echo "  Installing libudev-dev..."
        sudo apt-get install -y libudev-dev
    fi
elif command -v pacman &>/dev/null; then
    if ! pacman -Qi systemd &>/dev/null; then
        echo "  Installing systemd (provides libudev)..."
        sudo pacman -S --noconfirm systemd
    fi
else
    echo "  WARNING: unknown package manager — ensure libudev/libudev-dev is installed"
fi

# ---------------------------------------------------------------------------
# 1. Toolchain
# ---------------------------------------------------------------------------
echo "==> Checking Rust toolchain..."
if ! command -v cargo &>/dev/null; then
    echo "  Installing Rust via rustup..."
    curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh -s -- -y --no-modify-path
    source "$HOME/.cargo/env"
fi

if ! rustup target list --installed | grep -q "$TARGET"; then
    echo "  Adding target $TARGET..."
    rustup target add "$TARGET"
fi

for tool in flip-link elf2uf2-rs; do
    if ! command -v "$tool" &>/dev/null; then
        echo "  Installing $tool..."
        cargo install "$tool"
    fi
done

# ---------------------------------------------------------------------------
# 2. Clone / update repo
# ---------------------------------------------------------------------------
if [[ ! -d "$REPO_DIR" ]]; then
    echo "==> Cloning inputmodule-rs..."
    git clone "$REPO_URL" "$REPO_DIR"
else
    echo "==> Updating inputmodule-rs..."
    git -C "$REPO_DIR" pull --ff-only
fi

# ---------------------------------------------------------------------------
# 3. Apply firmware patches
# ---------------------------------------------------------------------------
echo "==> Applying patches..."
cd "$REPO_DIR"
python3 "$SCRIPT_DIR/patch.py"

# ---------------------------------------------------------------------------
# 4. Clean + Build
# ---------------------------------------------------------------------------
echo "==> Cleaning previous build artifacts..."
cargo clean
echo "==> Building $BINARY (release)..."
cargo build --release -p "$BINARY"

UF2="$REPO_DIR/target/$TARGET/release/$BINARY.uf2"
if [[ ! -f "$UF2" ]]; then
    # elf2uf2-rs may not produce the .uf2 automatically; convert manually
    ELF="$REPO_DIR/target/$TARGET/release/$BINARY"
    echo "  Converting ELF → UF2..."
    elf2uf2-rs "$ELF" "$UF2"
fi

echo
echo "Build complete: $UF2"
echo
echo "==> Flash instructions:"
echo "  1. Put the panel into bootloader mode:"
echo "       a) Hold the DIP switch while inserting the panel, OR"
echo "       b) Send bootloader command:  python3 -c \""
echo "          import serial; s = serial.Serial('/dev/ttyACM0', 115200);"
echo "          s.write(bytes([0x32, 0xAC, 0x02]))\""
echo "  2. A USB drive named RPI-RP2 will appear."
echo "  3. Copy the UF2 file to that drive:"
echo "       cp '$UF2' /path/to/RPI-RP2/"
echo "  4. Repeat for the second panel."
