#!/usr/bin/env bash
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"

echo "=== Toad Offline Test Environment Setup ==="

# 1. Check or install CMake
if ! command -v cmake &> /dev/null; then
    echo "CMake not found in PATH."
    CMAKE_VERSION="3.30.3"
    CMAKE_TAR="cmake-${CMAKE_VERSION}-linux-x86_64.tar.gz"
    CMAKE_URL="https://github.com/Kitware/CMake/releases/download/v${CMAKE_VERSION}/${CMAKE_TAR}"
    
    mkdir -p "$HOME/.local"
    echo "Downloading CMake ${CMAKE_VERSION} to ~/.local..."
    curl -sL "$CMAKE_URL" | tar -xz -C "$HOME/.local" --strip-components=1
    
    export PATH="$HOME/.local/bin:$PATH"
    if ! command -v cmake &> /dev/null; then
        echo "Error: Failed to install CMake to ~/.local/bin"
        exit 1
    fi
    echo "CMake installed successfully to ~/.local/bin/cmake"
else
    echo "CMake is already installed: $(cmake --version | head -n 1)"
fi

# 2. Check or install vcpkg
if [ -z "$VCPKG_ROOT" ]; then
    if [ -d "$HOME/vcpkg" ]; then
        export VCPKG_ROOT="$HOME/vcpkg"
    fi
fi

if [ -z "$VCPKG_ROOT" ] || [ ! -f "$VCPKG_ROOT/vcpkg" ]; then
    VCPKG_DIR="$HOME/vcpkg"
    echo "vcpkg not found. Cloning into $VCPKG_DIR..."
    if [ ! -d "$VCPKG_DIR" ]; then
        git clone --depth 1 https://github.com/microsoft/vcpkg.git "$VCPKG_DIR"
    fi
    echo "Bootstrapping vcpkg..."
    "$VCPKG_DIR/bootstrap-vcpkg.sh" -disableMetrics
    export VCPKG_ROOT="$VCPKG_DIR"
fi

echo "vcpkg is ready at: $VCPKG_ROOT/vcpkg"
# 3. Check for 32-bit compilation support (gcc -m32 / g++ -m32)
echo "Checking for 32-bit multilib toolchain..."
if ! echo 'int main(){return 0;}' | gcc -m32 -x c - -o /dev/null 2>/dev/null; then
    echo "WARNING: 32-bit C compiler (gcc -m32) failed. You may need to install multilib support:"
    echo "  sudo apt install gcc-multilib g++-multilib"
elif ! echo 'int main(){return 0;}' | g++ -m32 -x c++ - -o /dev/null 2>/dev/null; then
    echo "WARNING: 32-bit C++ compiler (g++ -m32) failed. You may need to install multilib support:"
    echo "  sudo apt install gcc-multilib g++-multilib"
else
    echo "32-bit compilation toolchain is ready."
fi

echo ""
echo "Environment setup complete!"
echo "To persist in your current shell session, run:"
echo "  export PATH=\"\$HOME/.local/bin:\$PATH\""
echo "  export VCPKG_ROOT=\"$VCPKG_ROOT\""
echo "  export VCPKG_DEFAULT_TRIPLET=\"x86-linux\""

