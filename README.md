# ![Officine Robotiche][Logo] µNAV
This is a project in development by [Officine Robotiche] to control motors.

# Release
Download last stable release

# Firmware features
- High speed DC motor control **1KHz**
- High speed serial communcation **115200bps**

# DC Motor control
- Velocity PID control
- Encoders management (A and B channels) 
- Motors current measurement
- Enable H-Bridge

## Communications
- OR-B Library communication
  - You can send different type of messages to control or set board

## Boards compliant
- [uNAV](https://github.com/officinerobotiche/uNAVPCB)
- [RoboController](http://tuttoelettronica.net/archives/455)
- [Motion Control](http://raffaello.officinerobotiche.it/schede-elettroniche/motion-control/)

#Tech
This repository follow **[Git flow](https://www.atlassian.com/git/tutorials/comparing-workflows/gitflow-workflow)** policy:
![Gitflow](https://raw.githubusercontent.com/quickhack/translations/master/git-workflows-and-tutorials/images/git-workflows-gitflow.png)

- **Master** branch stores the official release history
- **Develop** branch serves as an integration branch for features
- **Feature** branches use *develop* as their parent branch
- **Release-*** No new features can be added after this point—only bug fixes
- **Hotfix** branches are used to quickly patch production releases

## Communication libraries
- [C++](https://github.com/officinerobotiche/orblibcpp)
- [Java](https://github.com/officinerobotiche/orblibjar)
- [ROS (Robotic Operative System)](https://github.com/officinerobotiche/serial_bridge)
 
## Software
- [PID tuning](https://github.com/officinerobotiche/uNav_PID_Tuner)

[Officine Robotiche]:http://www.officinerobotiche.it/
[Logo]:http://2014.officinerobotiche.it/wp-content/uploads/sites/4/2014/09/ORlogoSimpleSmall.png
