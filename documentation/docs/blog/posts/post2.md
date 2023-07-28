---
title: Setting Up MPLAB X
Author: Carlos Espinosa
date: 2023-06-28
category: Tutorial
tags:
  - MPLAB X
  - Software
---

Problem with MPLAB X ?

---

## No Such file or directory for include files

- File -> Project Properties -> xc32-gcc

![Image](../../assets/images/TroubleShooting/MPLABX/MPLABX1.png)

- Choose Preprocessing and messages in the option categories
- Click the three dots next to include directories

![Image](../../assets/images/TroubleShooting/MPLABX/MPLABX2.png)

- Enter the relative folder path and click OK to save

- A heap is required, but has not been specified.
  - File -> Project Properties -> xc32-ld
  - Enter 0 bytes as the heap size

![Image](../../assets/images/TroubleShooting/MPLABX/MPLABX3.png)

- Click OK to save
