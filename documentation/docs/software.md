# Setup

## Cloning the Repository

To clone the Repo just execute the following command on Gitbash or your terminal prompt.
We are making the project folder in Desktop you can clone it into directory you want.

```bash
Clone https://github.com/uccross/open-source-autonomous-vehicle-controller
```

Now navigate to the modules directory in the Cloned Repo

```bash
cd open-source-autonomous-vehicle-controller/modules
```

Clone the submodules in this directory

```bash
Clone submodule https://github.com/mavlink/c_library_v2
```

## Setting up the MPLAB X

**Images are required for this steps**

File -> Open Project -> open-source-autonomous-vehicle-controller -> lib -> Serial.X

Open Serial.X Project Properties (File -> Project Properties)

Choose Connected Hardware Tool to PICkit3

Navigate to xc32-ld -> choose General as the option category -> set the Heap Size (bytes) to be 8000 bytes

Click OK to save the modified Project Properties

Running SerialM32.c

Click Clean and Build

Click Debug Project

Open a serial port terminal app (CoolTerm for Mac, PuTTY for Windows)

Output should be:

Try typing something and press Enter

What you type should now stay in the serial terminal
