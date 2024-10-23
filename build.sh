#!/bin/sh
current_directory="$(cd "$(dirname "$0")" && pwd)"
project_root_dir=$current_directory
# if on ICDS ROAR you will need to load modules to make this work
# module load cmake
# module load gcc/9.1.0

# if on Windows and using MSYS, might need a couple of tools
# https://code.visualstudio.com/docs/cpp/config-mingw
# pacman -S cmake
# pacman -S git
# pacman -S --needed base-devel mingw-w64-ucrt-x86_64-toolchain
# pacman -S mingw-w64-ucrt-x86_64-gnuplot

# if on MacOS, you will need to install cmake, gnuplot, xcode-tools

echo "Hello Friend! Let's try and build this application together :D"

mkdir -p ${project_root_dir}/third_party
cd ${project_root_dir}/third_party
#compile MATPLOTPLUSPLUS
if [ ! -d "matplotplusplus" ]; then
    git clone https://github.com/alandefreitas/matplotplusplus
    cd ${project_root_dir}/third_party/matplotplusplus
    rm -rf build install
    mkdir -p build && mkdir -p install && cd build
    cmake -DCMAKE_INSTALL_PREFIX=${project_root_dir}/third_party/matplotplusplus/install -DCMAKE_BUILD_TYPE=Release ..
    cmake --build . -j 4
    cmake --install .
fi

# compile the rest of the application
cd ${project_root_dir}

# remove the build directory that has the current code in it
echo "deleting the BUILD directory"
rm -rf build

echo "make a new BUILD directory to start the compiling process"
mkdir -p build
cd build

echo "cmake engage!"
cmake ..

echo "convert this to an executable application -- let's go!!"
cmake --build .
cd ..
echo "declare success -- hooray!"

echo "running the executable with some default parameters"
echo "  using -t 60  for a simulation time of 60sec"
echo "  using -d 0.1 for a simulation timestep of 0.1sec"
echo "./build/NINT -t 60 -d 0.01 > results.txt 2>&1"
echo "  the 2>&1 redirects the stderr to a NULL so we don't see the gnuplot problems"
./build/NINT -t 60 -d 0.01 > results.txt 2>&1

echo "Check the 'results.txt' for helpful debugging information"