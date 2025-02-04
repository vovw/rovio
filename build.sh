#!/bin/bash

# Exit on error
set -e

# Create build directory
mkdir -p build

# Function to get kindr headers
get_kindr() {
    echo "Getting kindr headers..."
    if [ ! -d "kindr" ]; then
        git clone --depth 1 https://github.com/ANYbotics/kindr.git
    fi
}

# Check for brew on macOS
if [[ "$OSTYPE" == "darwin"* ]]; then
    if ! command -v brew &> /dev/null; then
        echo "Homebrew not found. Please install Homebrew first:"
        echo "  /bin/bash -c \"\$(curl -fsSL https://raw.githubusercontent.com/Homebrew/install/HEAD/install.sh)\""
        exit 1
    fi

    # Install dependencies using Homebrew if not present
    DEPS="eigen opencv yaml-cpp cmake"
    for dep in $DEPS; do
        if ! brew list $dep &>/dev/null; then
            echo "Installing $dep..."
            brew install $dep
        fi
    done

    # Optional: Install OpenGL dependencies if visualization is needed
    if [ "$1" == "--with-viz" ]; then
        VIZ_DEPS="glew freeglut"
        for dep in $VIZ_DEPS; do
            if ! brew list $dep &>/dev/null; then
                echo "Installing $dep..."
                brew install $dep
            fi
        done
        CMAKE_ARGS="-DMAKE_SCENE=ON"
    fi
fi

# Get kindr headers
get_kindr

# Configure and build ROVIO
cd build
echo "Configuring with CMake..."
cmake .. \
    -DCMAKE_BUILD_TYPE=Release \
    -DKINDR_INCLUDE_DIR=../kindr/include \
    ${CMAKE_ARGS}

# Build
echo "Building..."
cmake --build . -j$(sysctl -n hw.ncpu)

echo "Build complete!"
echo "You can find the executable at: build/rovio_standalone" 