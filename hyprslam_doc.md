# HYPRSLAM build

## Build and install Pangolin

```bash
git clone --recursive https://github.com/stevenlovegrove/Pangolin.git
cd Pangolin/
git checkout v0.6
./scripts/install_prerequisites.sh recommended
cmake -B build
cmake --build build
cd build/
sudo make install
```

## Build and install ORB_SLAM3

```bash
git clone TODO
cd ORB_SLAM3/
export PANGOLIN_DIR=~/Pangolin
export LD_LIBRARY_PATH=$PANGOLIN_DIR/build/src:$LD_LIBRARY_PATH
export CMAKE_PREFIX_PATH=$PANGOLIN_DIR/build:$CMAKE_PREFIX_PATH
export PKG_CONFIG_PATH=$PANGOLIN_DIR/build:$PKG_CONFIG_PATH
./build.sh
```

## Rebuild HYPRSLAM only and/or add libs in /data/shared/

```bash
./hypr_build.sh
```

## Test HYPRSLAM

```bash
python3 test_hyprslam.py
```
