### This project is simple realtime demonstration of FSCF algorythm
If you are beginer, I suggest to read something about the magnetometers used in MEMS sensors. Its really important to know, how to calibrate them. The topic of fusion filters could be hard for you, but this filter can be used as module in you project. Remember failure is the mother of success.

### Demo video
https://www.youtube.com/watch?v=8DWVKc6RNK8&ab_channel=JosefJusta

### Hardware used:
-BOSCH sensor unit BMX055 (3x acc, 3x mag, 3x gyr)<br>
-Arduino Pro mini 3.3V (clone)<br>
-FTDI - FT232 for arduino connection<br>
-Few wires + bit of soldering skill<br>

<p align="center">
  <img src="https://raw.githubusercontent.com/Josef4Sci/AHRS_Filter/master/Pictures/DSC_0635.jpg" width="600" title="Screen1">
</p>

### Wiring
SCL, SDA - Just look at you arduino I2C pins.
BMX055: PS="1", SDO1="0", SDO2="0", CSB3="0", CSB1="NC", CSB2="NC"

