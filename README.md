# Autonomous Drone Landing on moving platform

This project is using a custom VTOL with PixHawk as a flight controller. Raspberry Pi 4 had been use as an On-Board processor to detect the specific tag by using vision computation. The control of the drone using MavSDK python.

- [![VIDEO 1](https://img.youtube.com/vi/w8bV-NbThaw/0.jpg)](https://www.youtube.com/watch?v=w8bV-NbThaw)
- [![VIDEO 2](https://img.youtube.com/vi/XmQxOzOqzXs/0.jpg)](https://www.youtube.com/watch?vXmQxOzOqzXs)

# Installing Ubuntu Mate on Raspberry Pi 4 from windows

1. Download the Ubuntu Image from https://ubuntu-mate.org/ports/raspberry-pi/
2. Download Etcher (a tools to flash the image) from https://www.balena.io/etcher/

## OS Configuration

1. Set [autologin](https://askubuntu.com/questions/1202230/auto-login-enable-on-turn-on-system) 
2. Disable monitor off
3. Set fix IP
4. Set ssh
````
sudo apt install openssh-server
sudo apt install sshguard (optional)
````

## Compile open cv

1. [Reference 2](https://www.philipzucker.com/installing-opencv-3-aruco-raspberry-pi-3/)
````
sudo apt-get update
sudo apt-get upgrade
sudo apt-get install -y build-essential cmake pkg-config
sudo apt-get install -y libjpeg-dev libtiff5-dev libjasper-dev libpng12-dev
sudo apt-get install -y libavcodec-dev libavformat-dev libswscale-dev libv4l-dev
sudo apt-get install -y libxvidcore-dev libx264-dev
sudo apt-get install -y libgtk2.0-dev
sudo apt-get install -y libatlas-base-dev gfortran
sudo apt-get install -y python2.7-dev python3-dev
cd ~
git clone https://github.com/opencv/opencv.git
git clone https://github.com/opencv/opencv_contrib.git
pip install numpy
cd ~/opencv
mkdir build
cd build
cmake -D CMAKE_BUILD_TYPE=RELEASE \
    -D CMAKE_INSTALL_PREFIX=/usr/local \
    -D INSTALL_PYTHON_EXAMPLES=ON \
    -D OPENCV_EXTRA_MODULES_PATH=~/opencv_contrib/modules \
    -D BUILD_EXAMPLES=ON ..

make -j4
sudo make install
sudo ldconfig
````
2. [Reference 1](https://docs.opencv.org/master/d7/d9f/tutorial_linux_install.html)

# Installing MAVSDK Python
````
pip install mavsdk
````
1. [Reference 1](https://github.com/mavlink/MAVSDK-Python)

# Installing VS Code
1. Need to manual download to home directory. Choose installer for debian ARM. Extension armhf.deb. [Download](https://code.visualstudio.com/#alt-downloads)
![Select Deb ARM64](https://pimylifeup.com/wp-content/uploads/2018/09/Downloading-ARM-deb-package-for-Visual-Studio-Code.jpg)
3. Double click and install using the package installer. The package installer must run using sudo
4. [Other Reference](https://pimylifeup.com/raspberry-pi-visual-studio-code/)

# Pixhawk connectivity
1. check the serial port
````
dmesg

[94896.408420] usb 1-1.2: Product: PX4 BL FMU v2.x
[94896.408437] usb 1-1.2: Manufacturer: 3D Robotics
[94896.408453] usb 1-1.2: SerialNumber: 0
[94896.695029] cdc_acm 1-1.2:1.0: ttyACM0: USB ACM device
[94896.698704] usbcore: registered new interface driver cdc_acm
[94896.698711] cdc_acm: USB Abstract Control Model driver for USB modems and ISDN adapters
[94901.200237] usb 1-1.2: USB disconnect, device number 8
[94901.429086] usb 1-1.2: new full-speed USB device number 9 using xhci_hcd
[94901.538362] usb 1-1.2: New USB device found, idVendor=1209, idProduct=5741, bcdDevice= 2.00
[94901.538375] usb 1-1.2: New USB device strings: Mfr=1, Product=2, SerialNumber=3
[94901.538386] usb 1-1.2: Product: fmuv2
[94901.538396] usb 1-1.2: Manufacturer: ArduPilot
[94901.538406] usb 1-1.2: SerialNumber: 3D0033000551363231393630
[94901.543522] cdc_acm 1-1.2:1.0: ttyACM0: USB ACM device

````

# Installing Mission Planner on Linux

While I prefer to use QGroundControl as my ground station, there are still reasons now and again that only Mission Planner can do the job. Mission Planner does not have native support for linux, but it is still possible to use via mono.

## Video Tutorial at https://youtu.be/XIS-nPs8Oq4

### On Linux

#### Requirements

Those instructions were tested on Ubuntu 18.04.
Please install Mono, either :
- ` sudo apt install mono-runtime libmono-system-windows-forms4.0-cil libmono-system-core4.0-cil libmono-winforms2.0-cil libmono-corlib2.0-cil libmono-system-management4.0-cil libmono-system-xml-linq4.0-cil`

or full Mono :
- `sudo apt install mono-complete`

#### Launching

- Get the lastest zipped version of Mission Planner here : https://firmware.ardupilot.org/Tools/MissionPlanner/MissionPlanner-latest.zip
- Unzip in the directory you want
- Go into the directory
- run with `mono MissionPlanner.exe`

You can debug Mission Planner on Mono with `MONO_LOG_LEVEL=debug mono MissionPlanner.exe`
