# hardware-auv8-power-management

This repo contain the firmware for both the PSU of AUV8, AUV8.1 and AUV lite.

> [!CAUTION]
> __Very Important__: There IS a difference between AUV8/8.1 and AUV lite. The enable pin for motor 2 is not the same. AUV8/8.1 uses the pin A0 while AUV lite uses pin PD_9. The pin to use is the file `include/pinDef.h` (Search for the definition of `MTR2`). Please note that this change is due to a defect on the PCB used on AUV8/8.1 which prevent us from using PD_9. The PCB are otherwise identical and could therefore be swapped. The PCB which should use A0 can be recognized by the presence of black wire that connect the PIN A0. Please make sure that correct pin is associated with `MTR2` in `include/pinDef.h` before programming any PCB.

## Mbed studio setup
* Clone this repo in a folder that you want to use as workspace for Mbed Studio: `git clone git@github.com:sonia-auv/hardware-auv8-power-management.git`. You may have to setup your ssh key before. (Note: if you don't want setup a ssh key AND you don't plan on updating+pushing your code, you may instead use the https mode: `git clone https://github.com/sonia-auv/hardware-auv8-power-management.git`)
* Open the cloned repo: cd hardware `cd hardware-auv8-power-management`
* Checkout the right branch:  `git checkout develop_fix_mbed` 
* This repo contains git submodule. After cloning this repo, pull the submodule by running the following commands in the root of the repo : `git submodule init` then `git submodule update` this command make take while to execute. Note: the submodules are `hardware-utility` in `include\` and `mbed-os` in the root. Mbed-os has a pin to a specific version (specifically `mbed-os-5.15.3-rc1` which match the commit `dfcb61e` in the official Mbed-os repo). This version should not be changed
* Make sure that you have downloaded the latest version of Mbed studio
* Open mded studio
* Open the workspace `File > Open workspace` and chose the folder in which you cloned to the repo (don't chose the folder of the current repo, chose the parent )
* The project should appear in the file explorer of Mbed Studio
* In the file explorer of Mbed Studio, right-click on the name of the project and chose `Set as active project`
* On the top-left corner, the field for `Target` should be empty. Choose `NUCLEO_F413ZH`
* You should now be ready to compile. Click on the harmer icon on the left panel to build the program. This will take a while to compile
* If the compile is successful, you should be ready to program the MCU
* To program the MCU, connect a ST-Link to your computer and the MCU board
* Power on the board (in the case of the MCU board of the PSU, you can use the barrel connector on the board to power it)
* In mbed Studio, choose the ST-Link in the target field in the top left corner of the left panel if it is not the case all ready
* Click on the play icon in the left panel to program the MCU. This will take a while. First the memory of the MCU will be wipe, then the MCU will be programed

> [!NOTE]
> Keep in mind that project is currently setup to use a submodule for mbed-os. This means that _every_ copy of the project that you on your computer will download a copy of mbed-os which contain about 750MB of data. Avoid having to many copies of the project. Alternatively, the folder mbed-os could be replace by a symlink that point to a share version of mbed-os, but this would complexify the setup. If you want to use a symlink, make sure to use the correct version of Mbed-OS (`mbed-os-5.15.3-rc1`)