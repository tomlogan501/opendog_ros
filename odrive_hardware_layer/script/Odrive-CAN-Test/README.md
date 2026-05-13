# Odrive-CAN-Test
ODrive motor control (without encoder) over CAN bus using SocketCAN — DSD Tech SH-C30G adapter

ODrive v3.6 motor control over CAN bus using a DSD Tech SH-C30G USB-CAN adapter on Ubuntu 22.04.

This repo contains both Python and C++ scripts to calibrate, spin and read feedback from a BLDC motor using the ODrive CANSimple protocol over SocketCAN.

# Hardware

- ODrive v3.6-56V (firmware v0.5.6)
- BLDC Motor — 350KV, 7 pole pairs
- DSD Tech SH-C30G USB-CAN Adapter
- 24V DC Power Supply
- Ubuntu 22.04


The DSD Tech SH-C30G uses Candlelight firmware which means Linux recognizes it as a native CAN interface (`can0`) — no custom drivers or wrappers needed. This makes it much simpler to work with compared to serial-based adapters.


Make sure:
1. Boot switch on adapter is OFF (Work Mode)
2. R120 switch on adapter is ON(termination enabled)
3. CAN 120R DIP switch on ODrive is ON

## Setup

Run these commands one by one on your linux terminal:

sudo modprobe can
sudo modprobe can_raw
sudo modprobe gs_usb

sudo ip link set can0 up type can bitrate 1000000 #bit rate set to 1MBPS 

ip -details link show can0

candump can0

After this you can Run the python and c++ scrips for test



