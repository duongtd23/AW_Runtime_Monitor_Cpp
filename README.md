
## Installation
The installation process requires the Spot library. Please follow the instructions at https://spot.lre.epita.fr/install.html to install the Spot library on your system.
For example, to install Spot 2.14.3 on Ubuntu 22.04, you can use the following commands:

```bash
# Install dependencies
sudo apt install \
    autoconf automake libtool pkg-config make g++ \
    python3 python3-dev python3-pip python3-cffi \
    libbdd-dev libboost-all-dev libgraphviz-dev \
    libgmp-dev flex bison

# Download and unzip Spot 2.14.3
wget http://www.lre.epita.fr/dload/spot/spot-2.14.3.tar.gz
tar -xvzf spot-2.14.3.tar.gz

# Build and install Spot
cd spot-2.14.3
./configure
make -j$(nproc)
sudo make install
```

We also need to install `libglm-dev`:
```
sudo apt install libglm-dev
```

After that, you can proceed to build the `aw_runtime_monitor` package.
```bash
# Clone the repository
git clone https://github.com/duongtd23/AW_Runtime_Monitor_Cpp.git
cd AW_Runtime_Monitor_Cpp

# Need to source autoware first.
# Adjust the path according to where Autoware is installed on your PC.
source ~/autoware/install/setup.bash

# Build the package
colcon build --symlink-install
```

## Usage
To run the monitor without planning shield:
```
ros2 launch aw_runtime_monitor monitor.launch.py output_path:=<your-path>
```

By default, the shield is disabled. The desired path to save trace file should be passed to `output_path` param.

To run the tool with planning shield enabled:
```
ros2 launch aw_runtime_monitor monitor.launch.py planning_shield_enabled:=true output_path:=<your-path>
```

