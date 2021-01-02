# I. RMD_X8_PRO_CAN_BUS_CONTROLER
Control RMD X8 PRO motor by CAN

# II. COMMAND TO SET AND READ MOTOR STATUS

## 1. Setting command
 ### SP_100  : Set speed 100 revolutions per minute (rpm)
 ### TL_450  : Turn clockwise 450 degree
 ### TR_180  : Turn counter clockwise 180 degree

## 2. Reading command
 ### MS      : Read motor status
 Expected output:
 #### + Motor temperature: temperature value
 #### + Motor torqe current: torque current value
 #### + Motor speed rpm: speed value
 #### + Motor encoder: encoder value

 ### MT      : Read motor multi turn angle
 Expected output:
 #### + Motor multi turn angle: angle 
 
 ### PI      : Read motor PID data
 ####  Expected output:
 #### Angle kp  :100
 #### Angle ki  :100
 #### Speed kp  :50
 #### Speed ki  :40
 #### Torque kp :50
 #### Torque ki :50
