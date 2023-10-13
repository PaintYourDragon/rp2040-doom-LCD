rm -rf rp2040-build
mkdir rp2040-build
cd rp2040-build

cmake -DCMAKE_BUILD_TYPE=MinSizeRel -DPICO_BOARD=vgaboard -DPICO_SDK_PATH=/Users/pburgess/Git/pico-sdk -DPICO_EXTRAS_PATH=/Users/pburgess/Git/pico-extras ..
