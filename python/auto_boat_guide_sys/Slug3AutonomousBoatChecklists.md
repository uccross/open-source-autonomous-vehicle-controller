# Slug 3 Autonomous Boat Checklists
Pavlo Vlastos, October 9th, 2021

### Brief
This document has three lists, a pre-flight checklist, a pre-launch checklist, and a post-launch checklist. The pre-flight checklist is for ensuring system readiness before departing to the location where the launch will happen. The pre-launch checklist is for checking all necessary a sytem connections and that the vehicle is safe for launch.

## Pre-Flight Checklist
- [ ] Duck-tape
- [ ] Razor blade
- [ ] Charge and bring battery >= x1
- [ ] Battery bag (Lipo-safe charging bag AKA 'bomb' bag)
- [ ] Remote controller
- [ ] Screw Driver (small and large philips and flat-head)
- [ ] Someone to help with the boat launch
- [ ] Phone (Charged)
- [ ] Wallet
- [ ] Keys 
- [ ] Multi-meter
- [ ] Towel x2 
- [ ] Small bench to prop the boat on
- [ ] Swimming shorts or change of shorts

## Pre-Launch Checklist
- [ ] Ensure the battery is disconnected
- [ ] USB connection from Raspberry Pi to Max32 microcontroller
- [ ] USB C connection from Drok buck-boost regulator to Raspberry Pi
- [ ] GPS to Max32 Purple Daughter Board 
  * 3V3 -> 3V3 **J21** (NOT A TYPO: the SAM-M8Q uses 3V3) pin 1 
  * RX  -> TX J12 pin 2
  * TX  -> RX J12 pin 3
  * GND -> GND J21 pin 6
- [ ] IMU connected to Max32 Purple Daughter Board
- [ ] Radio transceiver connected to J14
  * Red wire    -> 5V pin 2 J14 
  * Black wire  -> GND pin 1 
  * Green wire  -> U5Rx pin 4 J14
  * White wire  -> U5Tx pin 5 J14
  * Yellow wire -> Nothing
- [ ] Primary 3-pin connector for Max32 Purple Daughter Board 5V Rail from Drok buck-boost regulator (Channel 1)
- [ ] 3-pin connector for right motor ESC (Channel 2) GND (black) are the pins closest to the board. 5V are pins in the middle. PWM signals are the pins farthest from the board (tallest).
- [ ] 3-pin connector for left motor ESC (Channel 3)
- [ ] 3-pin connector for rudder servo (Channel 4)
- [ ] Secondary 3-pin connector for Max32 Purple Daughter Board 5V Rail from extra ESC (Channel 9)
- [ ] Ensure the main emergency power button is off
- [ ] Connect and secure battery
- [ ] Power on the Slug 3 with the main emergency power button
- [ ] Connect to WiFi hotspot with a computer. 
  * Network name: SlugNetwork, 
  * password: Bananas!
- [ ] SSH into pavlo@10.0.42.1 (double check with arp -a command in terminal), password: pavlo
- [ ] cd to Repos/open-source-autonomous-vehicle-controller/python/auto_boat_guide_sys/
- [ ] sudo python3 main_guide_sys.py -c "/def/ttyUSB1" --csv_file /media/pavlo/PGV/log_2021_10_9.csv --log_file /media/pavlo/PGV/log_2021_10_9.log -e --ecom "/dev/ttyUSB0" 
  * The dates for the files should be changed accordingly
  * see -h option for more, such as -d for debug, -m for mode, and others
  * The usb ports might switch, so just swap ttyUSB0 with ttyUSB1 if the related error shows up, and double check connections
- [ ] Test remote controls
- [ ] control c (^c) and check that the data logger worked over ssh
- [ ] re-run sudo python3 main_guide_sys.py -c "/def/ttyUSB1" --csv_file /media/pavlo/PGV/log_2021_10_9.csv --log_file /media/pavlo/PGV/log_2021_10_9.log -e --ecom "/dev/ttyUSB0" 
- [ ] Have someone place the boat in the water while the other person holds the remote controller
- [ ] Remote control the boat to an okay starting location
- [ ] Then, either keep controlling manually or switch to autonomous mode

## Post-Launch Checklist
- [ ]  Bring the boat back to launch location either with remote control, or wait for it to comeback autonomously (assuming a waypoint was set for home)
- [ ]  If in range of the Wifi hotspot, and ssh-ed in, control c (^c) to quit the guidance and logging python script
- [ ] Push the emergency power button on the top
- [ ] Take the boat out of the water.