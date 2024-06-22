# rbkairos_darko_packages

## Installation

Clone the repository:

```bash
git clone --recurse-submodule https://github.com/RobotnikAutomation/rbkairos_darko_packages
```

Make sure you have installed all dependencies before compiling and running all the software.

### Dependencies

For generic dependencies of the workspace, cd to the workspace and:

```bash
rosdep install --from-path src --ignore-src -r -y
```

To install Robotnik's private packages, cd to deb folder and:

```bash
./install_debs.sh
```

Franka Emika dependencies:
- libfranka requires realtime kernel, follow its [guide](https://frankaemika.github.io/docs/installation_linux.html) to set up realtime kernel

- Installing ros-melodic-franka autoinstall the necessary dependencies
```bash
sudo apt-get install ros-melodic-melodic-franka
```

Basler camera dependencies:
- Download pylon camera software from basler's [website](https://www.baslerweb.com/en/sales-support/downloads/software-downloads)
- Install the library:
```bash
sudo dpkg -i pylon_6.3.0.23157-deb0_amd64.deb
```
- Add pylon path to .bashrc
```bash
echo "export PYLON_ROOT=/opt/pylon" >> ~/.bashrc
```

- Add pylon-ros-camera dependencies to rosdep lists
```bash
sudo sh -c 'echo "yaml https://raw.githubusercontent.com/basler/pylon-ros-camera/master/pylon_camera/rosdep/pylon_sdk.yaml" > /etc/ros/rosdep/sources.list.d/30-pylon_camera.list' && rosdep update
```

- Install dependencies:
```bash
rosdep install --from-paths . --ignore-src --rosdistro=$ROS_DISTRO -y
```

Azure Kinect DK dependencies:
- Adds Microsoft repositories:
```bash
curl -sSL https://packages.microsoft.com/keys/microsoft.asc | sudo apt-key add - && sudo apt-add-repository https://packages.microsoft.com/ubuntu/18.04/prod && sudo apt-get update
```

- For ARM architectures:

```bash
curl -sSL https://packages.microsoft.com/keys/microsoft.asc | sudo apt-key add - && sudo apt-add-repository https://packages.microsoft.com/ubuntu/18.04/multiarch/prod && sudo apt-get update 
```
- Install packages:
```bash
sudo apt-get install libk4a1.4* k4a-tools
```
- Adds udev rules:
```bash
cd /etc/udev/rules.d/ && sudo wget https://raw.githubusercontent.com/microsoft/Azure-Kinect-Sensor-SDK/develop/scripts/99-k4a.rules
```
