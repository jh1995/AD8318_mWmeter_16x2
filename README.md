# AD8318_mWmeter_16x2
Arduino based milliWatt meter with AD8318 and I2C 16x2 display.

This in an adaptation for AD8318 by Paolo IK1ZYW of PA0RWE work (https://pa0rwe.nl/?page_id=356) originally meant for AD8317.

CHANGES with reference to PA0RWE version:
* Slope and calibration points have been adjusted from AD8317 to AD8318.
* I use LiquidCrystal_PCF8574 library because there are too many incompatible LiquidCrystal_I2C libraries around!
* Changed the banner in Menus.ino to reflect this is a different firmware

WARNING. I2C address of the display module is around line 142. Change it accordingly! Most "popular" values for PCF8574(A) adapters are 0x20, 0x27, 0x3F.

For all the rest refer to the original comment in the main .ino file.


