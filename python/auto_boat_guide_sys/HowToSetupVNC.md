# How to setup a Raspberry Pi 4B to do VNC
Download Real VNC, others may work with the instructions below, but I haven't tested them.

You will want to download the Real VNC client viewer for your computer that will be connecting to the raspberry pi (RPI4) hotspot. See the HowTo document for more information on setting up the WiFi hotspot.

With the RPI4 connected to the internet and using a montior, keyboard, and mouse (unless you want to use ssh to install everything, or similar), follow the instructions in this link: 
https://forums.raspberrypi.com/viewtopic.php?t=288769

And this one too: https://www.youtube.com/watch?v=LTVtKiz2KGE&t=0s

Then:

sudo apt-get install xserver-xorg-video-dummy

From: https://askubuntu.com/questions/453109/add-fake-display-when-no-monitor-is-plugged-in

Found a way to do it without requiring a dummy plug: Ubuntu Headless VNC VESA 800x600 Resolution Fix ~ Andy Hawkins @ June 12, 2011

Basically install a dummy driver:

sudo apt-get install xserver-xorg-video-dummy

Then write it in the /usr/share/X11/xorg.conf.d/xorg.conf (or possibly /etc/X11/xorg.conf) file (create one, if it does not exist):

Section "Device"
    Identifier  "Configured Video Device"
    Driver      "dummy"
EndSection

Section "Monitor"
    Identifier  "Configured Monitor"
    HorizSync 31.5-48.5
    VertRefresh 50-70
EndSection

Section "Screen"
    Identifier  "Default Screen"
    Monitor     "Configured Monitor"
    Device      "Configured Video Device"
    DefaultDepth 24
    SubSection "Display"
    Depth 24
    Modes "1024x800"
    EndSubSection
EndSection

#### Important steps after following the above instructions:
If you want to use a regular monitor make sure to rename the xorg.conf:
1. cd /usr/share/X11/
2. **sudo mv xorg.conf xorg.conf.bak**
3. sudo reboot

If you want to switch from a real monitor back to a virtual monitor:
1. cd /usr/share/X11/
2. **sudo mv xorg.conf.bak xorg.conf**
3. sudo reboot

Please note the .bak is indicating that this is a backup file. You could name it something else other than *xorg.conf* so that it doesn't activate the dummy monitor/display

***OR use the one found here:
https://techoverflow.net/2019/02/23/how-to-run-x-server-using-xserver-xorg-video-dummy-driver-on-ubuntu/
:
Section "Monitor"
  Identifier "Monitor0"
  HorizSync 28.0-80.0
  VertRefresh 48.0-75.0
  # https://arachnoid.com/modelines/
  # 1920x1080 @ 60.00 Hz (GTF) hsync: 67.08 kHz; pclk: 172.80 MHz
  Modeline "1920x1080_60.00" 172.80 1920 2040 2248 2576 1080 1081 1084 1118 -HSync +Vsync
EndSection
Section "Device"
  Identifier "Card0"
  Driver "dummy"
  VideoRam 256000
EndSection
Section "Screen"
  DefaultDepth 24
  Identifier "Screen0"
  Device "Card0"
  Monitor "Monitor0"
  SubSection "Display"
    Depth 24
    Modes "1920x1080_60.00"
  EndSubSection
EndSection