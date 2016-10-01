# ![Officine Robotiche][Logo] - µNav [![Build Status](https://travis-ci.org/officinerobotiche/uNAV.X.svg?branch=feature%2Ftravis-CI_integration)](https://travis-ci.org/officinerobotiche/uNAV.X) [![Stories in Ready](https://badge.waffle.io/officinerobotiche/uNAV.X.png?label=ready&title=Ready)](http://waffle.io/officinerobotiche/uNAV.X) [![Join the chat at https://gitter.im/officinerobotiche/uNAV.X](https://badges.gitter.im/Join%20Chat.svg)](https://gitter.im/officinerobotiche/uNAV.X?utm_source=badge&utm_medium=badge&utm_campaign=pr-badge&utm_content=badge)

This is a project in development by [Officine Robotiche] to control motors.

### **The [µNav board](https://github.com/officinerobotiche/uNAVPCB) is HERE!**

# Release
- [**Download last stable release**](https://github.com/officinerobotiche/uNAV.X/releases)
- [Wiki] with all detailed information about firmware
- Feel free to ask for help, submit suggestions, satisfy your curiosity on our [**Users Group**](https://groups.google.com/forum/?hl=it#!forum/unav-users)

# Firmware features
- High speed DC motor control **1KHz**
- High speed serial communication **115200bps**
- Interrupt based
- Control EEPROM onboard
- Dynamic configure all parameters on boards

## DC Motor control
- Velocity PID control
- Encoders management (A and B channels) 
- Motors current measurement
- Enable H-Bridge

## Communications
- *[OR-Bus]* Library communication for **serial communication** 
  - You can send different type of messages to control or set board
- No blocking **I2C** Communication 
- [ROS interface](https://github.com/officinerobotiche/ros_orbus_interface) for robotics application

## Boards compliant
- [µNav](http://rnext.it/project/unav)
- [RoboController](http://tuttoelettronica.net/archives/455)
- [Motion Control](http://rnext.it/project/board/motion-control/)

[OR-Bus]:https://github.com/officinerobotiche/or_bus_c.X.git
[Wiki]:https://github.com/officinerobotiche/uNAV.X/wiki
[Officine Robotiche]:http://www.officinerobotiche.it/
[Logo]:http://2014.officinerobotiche.it/wp-content/uploads/sites/4/2014/09/ORlogoSimpleSmall.png
