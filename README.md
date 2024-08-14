# Installation: 
Important notes before you start:
* Everything below assumes you are compiling for Pixhawk Cube Orange (although other configurations may also work if you adjust the source code accordingly).
* When configuring "UAV Toolbox Support Package for PX4 Autopilots", you should select:  
  * Design Flight Controller in Simulink (which is the default option for "Simulink application").
  * PX4 Cube Orange as your PX4 Autopilot board (this is not by default).
  * cubepilot_cubeorange_default as your "Build Target" (should be the default).
  * Keep the default startup script (I recommend completely erasing the SD card if you have it).
* Prepare your hardware now:
  * Remove the SD card, and format it (FAT32). You can keep it unplugged until you have completed all the installation steps below (SD card is not needed until you start configuring the firmware through QGC).
  * Do a factory reset for your Pixhawk Cube Orange and flush it with the default PX4 firmware (can be the most recent).


## Pre-requisites:
You should follow standard [MATLAB R2023a](https://www.mathworks.com/help/releases/R2023a/supportpkg/px4/setup-and-configuration_buiyb9j-1.html) 
documentation for all installation steps, unless stated otherwise. A quick recap is given below:
* Install MATLAB R2023a (R2023b might work, but was not tested)
* Install [MinGW](https://www.mathworks.com/support/requirements/supported-compilers.html)
* Install [UAV Toolbox Support Package for PX4 Autopilots](https://www.mathworks.com/help/releases/R2023a/supportpkg/px4/ug/install-support-for-px4.html):
  * Install [Python 3.8.2](https://www.mathworks.com/help/releases/R2023a/supportpkg/px4/ug/install-python-windows.html).
  * Install [Cygwin toolchain v0.8](https://www.mathworks.com/help/releases/R2023a/supportpkg/px4/ug/setup-cygwin-toolchain.html).
  * (Optional) I recommend (cloning PX4 into a separate, top-level directory)[https://www.mathworks.com/help/releases/R2023a/supportpkg/px4/ug/setting-px4-toolchain-windows.html], for example, D:/PX4/v1.13.3/Firmware.

## Install PX4-Simulink I/0 Upgrade
You must clone stock PX4 v1.12.3, and go through all of the default installation steps, before attempting anything further. 
Follow the documentation and make sure all of the stock functionality of the R2023a Matlab/Simulink has been installed and configured correctly (run examples!). 
I can't help you if you can't install/configure MATLAB/toolboxes or follow step-by-step MATHWORKS documentation. 

1. Locate the Firmware installation directory - this is where you have already installed PX4 v1.12.3, let's say D:/PX4/v1.13.3/Firmware.
2. Go to D:/PX4/v1.13.3/Firmware/src/drivers/ and delete these folders:
    * uavcan
    * uavcan_v1
    * uavcannode   
3. Locate the Firmware upgrade folder - this is the folder provided in this repository, called "Firmware"   
4. Install the upgrade by copying the contents of the Firmware upgrade folder into D:/PX4/v1.13.3/Firmware, replacing all files.
5. Recompile upgraded PX4 firmware. You need to re-run the setup app for "UAV Toolbox Support Package for PX4 Autopilots".
    * In Matlab, click on the "home" tab and use the dropdown menu under "Add-Ons" icon to select "Manage Add-Ons"
    * In the Add-On Manager, locate "UAV Toolbox Support Package for PX4 Autopilots"
    * Click the gear icon on the far right of the tab
    * Re-do UAV Toolbox Support Package for PX4 Autopilots setup every time you need to re-compile the PX4 firmware.

# Integration with Simulink
At this point, you should be able to compile the Simulink model and upload firmware to the board. You can also upload the firmware before compiling anything 
through Simulink to check the status of everything without the control system deployed. You should also be able to check the status of the sm_ctrl_mod through 
the MAVLINK shell in QGC. There should be no further action required on the firmware side, besides configuration.

Simulink integration is very application-dependent. This specific package was written to be used with the RRTV, which is also the only example/walkthrough for this work.
