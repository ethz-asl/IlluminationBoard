# IlluminationBoard
## Description
Strong triggerable LED Driver with ROS (or other environment) capability 

## Versions
 - IlluminationBoard V2 Rev. 1
 - IlluminationBoard V2 Rev. 2 & 3

## Firmware
### ONLY FOR V2 REV 1
This version has its own Arduino package, as well as firmware.
Include https://raw.githubusercontent.com/ethz-asl/IlluminationBoard/master/package_Illuminationboard_index.json in the Arduino IDE under "File>Preferences>Settings" in the "Additional Boards Manager URLs" section.

After doing this you can search it in the Board Manager and install it from there.

Use the firmware from the "old/IlluminationboardV2_00" folder.

### V2 REV 2 & 3 
As of revision 2 the MCU pinout has been changed. It is no longer required to install the above package, as it now works with the standard Arduino MKRZERO package.

Use the firmware from the "_latest" folder.

### ROS
See _latest/illuminationboard_v2_ros/datasheet/IlluminationBoard V2 - datasheet.pdf for more information on how to use the IlluminationBoard with ROSSerial.