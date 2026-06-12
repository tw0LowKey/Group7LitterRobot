# Project Software Installation Guide

## Prerequisites

This project is designed to run on a NVIDIA Jetson Orin Nano running **JetPack 6.2**.

---

## 1. Update the System

Update package lists and upgrade installed packages:

```bash
sudo apt update
sudo apt upgrade
```

---

## 2. Install the gs_usb CAN Module

The AgileX Scout Mini and PiPER Arm communicate over CAN using a USB-CAN adapter. Install the required gs_usb kernel module using the following steps.

### Install Dependencies

```bash
sudo apt install build-essential libncurses-dev -y
```

### Download the Builder Script

```bash
wget -q https://github.com/lucianovk/jetson-gs_usb-kernel-builder/raw/main/jetson-gs_usb-kernel-builder.sh
chmod +x jetson-gs_usb-kernel-builder.sh
```

### Build and Install the Module

```bash
sudo ./jetson-gs_usb-kernel-builder.sh
```

### Verify Installation

```bash
sudo modinfo gs_usb
```

If module information is displayed, the installation was successful.

### Load the Module

```bash
sudo modprobe gs_usb
```

If successful, no output will be shown.

---

## 3. Configure Persistent Device Names (Udev Rules)

Linux may assign different device names after every reboot or reconnect. To ensure the Scout Mini, PiPER Arm, and YDLIDAR always appear with the same names, create udev rules.

### Step 1: Identify Each Device

Connect one device at a time and identify its USB information:

```bash
lsusb
```

For serial devices such as the YDLIDAR, identify the device path:

```bash
udevadm info -a -n /dev/ttyUSB0
```

For CAN adapters, identify the USB serial number:

```bash
udevadm info -a -p $(udevadm info -q path -n can0)
```

Look for attributes such as:

```text
ATTRS{idVendor}
ATTRS{idProduct}
ATTRS{serial}
```

Record the values for each device.

---

### Step 2: Create a Udev Rules File

Create a custom rules file:

```bash
sudo nano /etc/udev/rules.d/99-robot.rules
```

Add rules similar to the following.

#### AgileX Scout Mini

```text
SUBSYSTEM=="net", ACTION=="add", ATTRS{serial}=="SCOUT_SERIAL_NUMBER", NAME="can_scout"
```

#### AgileX PiPER Arm

```text
SUBSYSTEM=="net", ACTION=="add", ATTRS{serial}=="ARM_SERIAL_NUMBER", NAME="can_arm"
```

#### YDLIDAR

```text
SUBSYSTEM=="tty", ATTRS{serial}=="YDLIDAR_SERIAL_NUMBER", SYMLINK+="ydlidar"
```

Replace the serial numbers with the values found in the prior step.

---

### Step 3: Reload Udev Rules

Reload and apply the rules:

```bash
sudo udevadm control --reload-rules
sudo udevadm trigger
```

Reconnect the devices or reboot the Jetson.

After reloading:

* Scout Mini CAN interface will appear as `can_scout`
* PiPER Arm CAN interface will appear as `can_arm`
* YDLIDAR will be accessible via `/dev/ydlidar`

---

## 4. Add CAN Helper Aliases

Append the following aliases to the end of your `~/.bashrc` file:

```bash
echo "" >> ~/.bashrc
echo "# CAN helper aliases" >> ~/.bashrc
echo "alias candown='sudo ip link set can_scout down; sudo ip link set can_arm down; echo \"Scout and Arm can ports are now down\"'" >> ~/.bashrc
echo "alias canup='sudo ip link set can_scout up type can bitrate 500000; sudo ip link set can_arm up type can bitrate 1000000 sample-point 0.875; echo \"Scout and Arm can ports are now running\"'" >> ~/.bashrc
```

Reload your shell configuration:

```bash
source ~/.bashrc
```

### Usage

Bring CAN interfaces online:

```bash
canup
```

Bring CAN interfaces offline:

```bash
candown
```

---

## Installation Complete

Your system should now have:

* JetPack 6.2 installed
* Updated system packages
* `gs_usb` CAN kernel module installed and loaded
* Persistent naming for:

  * AgileX Scout Mini (`can_scout`)
  * AgileX PiPER Arm (`can_arm`)
  * YDLIDAR (`/dev/ydlidar`)
* CAN management aliases available through `canup` and `candown`
