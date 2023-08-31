# Standard installation steps: 

## Clone the standard PX4 repo:
1. go to Cygwin installation folder (toolchain), run "run-console.bat".
2. in the console, go the location of where you want to install firmware (e.g. "/cygdrive/d/PX4/f/12.3/")
3. Clone px4 repo by running "git clone https://github.com/PX4/Firmware.git Firmware"
4. Enter firmware folder: "cd Firmware"
5. checkout v1.12.3: "git checkout v1.12.3"
6. fetch and update all submodules "git submodule update --init --recursive"
7. You can check if everything (for stock PX4) is installed properly by running the setup app of MATLAB.

## Modify MATLAB toolbox
I assume that your have installed MATLAB to default C drive.  
We need to modify the PX4 toolbox of MATLAB. To do that, copy the contents of the "MATLAB" folder 
into "C:\ProgramData\MATLAB\". 
This should be a hidden folder that already exists on your drive if you have installed MATLAB.

## Copy modified PX4 source files:
1. Delete "Firmware\src\drivers\uavcan" folder from the firmware (it has extra files that need to be gone)
2. Copy the "Firmware" folder to the source Firmware (the folder in which you have installed PX4 Firmware) and replace all files.
3. Re-run the MATLAB setup app to compile modified firmware. (setup app is in add-on manager)
4. Not all the functionality should work if you have not yet copied a new startup routine, but PX4 should boot.

# Modify the Firmware:
If you wish to make any changes to the firmware, it is recommended to always work on the local copy of all the files you are trying
to modify and then repeat "Standard installation steps" as needed.

## Add new PX4 parameters
Currently, any Simulink-related parameters are created and managed by "sim_ctrl_mod" module, located at 
"\Firmware\src\modules\sim_ctrl_mod\". If you wish to add or modify anything, proceed to that folder
for further instructions.