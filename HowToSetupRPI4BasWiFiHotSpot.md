# How to setup a Raspberry Pi 4B as a local WiFi hot-spot
[NOT DOWN YET - I AM STILL ADDING MORE DETAIL] By Pavlo Vlastos

## Brief
This guide explains how to setup the Raspberry 4B as a local WiFi hot-spot and why you might want to do that.

## Requirements
* A Raspberry Pi 4B (RPI4B)
* A microSD Card flashed with at least Ubuntu 21.04
    * Desktop version was tested to work, but ideally you should use a server image, because the gui (or Head) that the Desktop version has is unecessary. We will be ssh-ing into the RPI4B wirelessly. 
* Power cable for the RPI4B
* (optional, but recommended) A monitor
    * HDMI cable 
* (optional, but recommended) A keyboard
* (optional, but recommended) A mouse

## Steps
1. Plug in the microSD card to the RPI4B
2. Power on the RPI4B
3. Add a .service file to /etc/systemd/system (must be here for systemd services to work)
4. Add a bash script to /home/\<username>/SetupWiFiHotSpot.sh (or a locatino of you choice)

chmod the service and the .sh script

sudo systemclt enable the .service

make sure the RPI4B doesn't auto-connect to a known network

reboot

Connect to the WiFi network if it appears

attempt to ssh into the pi, for example: '$ ssh ubuntu@10.42.0.1'