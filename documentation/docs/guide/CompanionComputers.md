# Companion Computers

## Hardware

Required materials

- Raspberry Pi
  - Rpi 4 Model B 8GB RAM
  - Power supply: Raspberry Pi 15W USB-C Power Supply
- PC with microSD card slot
- MicroSD card
  - SanDisk 64GB Ultra MicroSDXC

![Hardware Image](../assets/images/CompanionComputers/CC1.jpg)

## Software

- Install the Raspberry Pi Imager [here.](https://www.raspberrypi.com/software/)

- Install the VNC Viewer [here.](https://www.realvnc.com/en/connect/download/viewer/)

## Imaging in Headless Mode

- Insert the microSD card into the PC
- Open the Raspberry Pi Imager
  - Select the Raspberry Pi OS (64-bit) as the Operating System
  - Select the microSD card as the Storage

![Rasberry Pi Imager](../assets/images/CompanionComputers/CC2.png)

- Click on the gear icon in the bottom right hand corner to access the Advanced options
  - Select Set hostname: raspberrypi.local
  - Select Enable SSH and Use password authentication
  - Select Set username and password
    - Type the username and password of your choice
  - Select Configure wireless LAN
    - Type your wifi network name as the SSID and your wifi network password as the Password
    - Find your country’s two-letter country code
  - Click Save

![Advance Options](../assets/images/CompanionComputers/CC3.png)

- Now, Click Write

## OSAVC

- Connect the OSAVC and PICkit3 Debugger to your PC
- Load TestHarness.X’s code onto the OSAVC
  - Open the TestHarness.X folder
  - Click Make and Program Device ![Save](../assets/images/CompanionComputers/CC.png)

## Raspberry Pi 4

- Insert the microSD card into the Raspberry Pi 4
- Connect the micro-USB port of the OSAVC to the USB port of the Raspberry Pi 4
- Power the Raspberry Pi 4 using the USB-C port with the power supply
- Open an SSH tunnel from your PC to the Raspberry Pi 4 by typing ssh [username]@[hostname].local in your PC terminal
  - sudo raspi-config
  - Select 5 Interfacing Options
  - Select P3 VNC
  - Click Yes to enabling the VNC server
  - Save your changes
- Open VNC Viewer and connect to [hostname].local at the top

![RealVNC Viewer](../assets/images/CompanionComputers/CC4.png)

- Install opencv and its dependencies
  - sudo apt-get update
  - sudo apt install python3-opencv
- Install pymavlink
  - sudo pip3 install pymavlink
- Install a serial terminal on the Raspberry Pi 4
  - sudo apt-get install minicom
  - minicom -b 115200 -o -D /dev/ttyUSB0
  - Press Ctrl-A Z to get the Mincom Command Summary dialog

![Mincom](../assets/images/CompanionComputers/CC5.png)

- Press o to Configure Minicom
- Select Serial port setup

![Serial Port  Setup](../assets/images/CompanionComputers/CC6.png)

- press F to set F - Hardware Flow Control : No

![Hardware Flow Control](../assets/images/CompanionComputers/CC7.png)

- Save by pressing Enter and exit the Minicom Command Summary
- Click on the OSAVC’s reset button
- Press h to transmit a Mavlink heartbeat message from the OSAVC to the Raspberry Pi 4 and then press q to quit

![Hardware Flow Control](../assets/images/CompanionComputers/CC8.png)

- Create a new terminal tab, delete the Minicom terminal tab
- In the new terminal, enter

```sh
git clone https://github.com/uccross/open-source-autonomous-vehicle-controller.git
```

```sh
cd open-source-autonomous-vehicle-controller
```

```sh
cd companion-computer
```

```sh
sudo python3 OSAVC_web_server_rpi.py
```

- Open http://0.0.0.0:80/ in Chromium by right clicking the link and clicking Open URL.

![Terminal](../assets/images/CompanionComputers/CC9.png)

- A webpage like this should appear

![Web Page](../assets/images/CompanionComputers/CC10.png)

- Click the Connect Mav button
  - The button should turn red
  - The terminal should output target_system 1, target component 0

![Target Component](../assets/images/CompanionComputers/CC11.png)

![Target Component](../assets/images/CompanionComputers/CC12.png)

- Finally, click the Connected button again, and the MAVstatus should become Connected

![Connected](../assets/images/CompanionComputers/CC13.png)
