# Building Your First Project

## Cloning the Repository

To clone the Repo just execute the following command on Gitbash or your terminal prompt.
We are making the project folder in Desktop you can clone it into directory you want.
Navigate to your desired directory

```bash
cd Desktop
```

```bash
git clone https://github.com/uccross/open-source-autonomous-vehicle-controller.git
```

Now navigate to the modules directory in the Cloned Repo

```bash
cd open-source-autonomous-vehicle-controller/modules
```

Clone the submodules in this directory

```bash
git clone https://github.com/mavlink/c_library_v2
```

## Setting up Hardware

![Connection](../assets/images/first_project/connection.jpg)

- Connect ICSP (in circuit serial programming) of PICKIT 3 to the OSAVC
  - Use a mini USB-b cable to connect the PICKIT 3 to your computer
- Use a micro USB cable to connect the OSAVC to your computer for power and a communications port

## Setting up Serial Terminal (CoolTerm / mobaxterm / putty)

- Open CoolTerm after connecting your PC to the OSAVC
- Choose the port at the bottom left of the application as usbserial
![usbSerial](../assets/images/first_project/usbSerial.png)

- Click Options -> Select 115200 as the baud rate -> Click OK to save
![baudrate](../assets/images/first_project/baudRate.png)

- Click Connect at the top of the application

## Setting up the MPLAB X

### Follow the below given setps for running Serial.X

- File -> Open Project
![Open Project](../assets/images/first_project/openProject.png)

- Now navigate to -> open-source-autonomous-vehicle-controller -> lib -> Serial.X
you would be able to see the below window with the serial project open
![Serial Project window](../assets/images/first_project/serial.png)

- Now Open Serial.X Project Properties (File -> Project Properties)
![Serial Project Properties](../assets/images/first_project/projectProperties.png)

- Choose Connected Hardware Tool to PICkit3

- Navigate to xc32-ld
![xc32-id](../assets/images/first_project/xc32id.png)

- Choose General as the option category
![General option category](../assets/images/first_project/general.png)

- set the Heap Size (bytes) to be 8000 bytes
![Heap Size](../assets/images/first_project/bytes.png)

- Click OK to save the modified Project Properties
![OK](../assets/images/first_project/ok.jpg)

- Click Clean and Build
![Clean and Build](../assets/images/first_project/CleanBuild.png)

- Click Make and Program Device
![Make and Program Device](../assets/images/first_project/MakeProgram.png)

- Open your preset serial terminal application (CoolTerm)

Output should be:
![Output from CoolTerm](../assets/images/first_project/outputCoolterm.png)

- Try typing something and press Enter
![Something](../assets/images/first_project/everything.png)

- What you type should now stay in the serial terminal
