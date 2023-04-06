## Autonomous Car
#### Description
This repository is for the Kalamzoo public high schools autonomous. Every year we compete in a competition called [Square One](http://www.squareonenetwork.org/). Although Square One has multiple diffrent competitions, we specificly compete in the [Autonomous Innovative Vehicle Design Challenge](http://www.squareonenetwork.org/innovative-vehicle-design/autonomous-innovative-vehicle-design-challenge/) Each year the task's are diffrent but the goal is the same, To re engineer a power jeep. This year we used three arduinos, One for led ie brake lights & flood light's, We have a one for the speakers (Yes we have speakers, and their bluetooth). And lastly we have our main arduino wich is for all of our motors and sensors. This year we went with a traditional jeep design aswell as traditional steering. 

#### Installation
For Installation you will need a few seperate projects. Each of them are maintained seperately from the project. But for this year project well be using

- [Navis](https://github.com/winters-brown/Navis)

Each repositorys go over how to install them and so forth on the intended operating system.

After installing those you will want to update and upgrade your system using the command

```
$ sudo apt update -y && sudo apt upgrade -y
```

You will now want to follow this [tutorial](https://www.raspberrypi.org/documentation/configuration/wireless/access-point.md)

##### Updating Arduino
The first thing you will want to do is to install the Encoder.zip library into your arduino ide. It is contained in this project as a zip under the ```lib\``` folder! If you are not sure how todo it or forgot read this [tutorial!](https://www.arduino.cc/en/guide/libraries)

#### Contributing / Updating
```
# Create Branch
git checkout -b Branch_Name

# Move To Branch
git checkout Branch_Name

# Pushing repository
git push origin Branch_Name
```

#### Credits
  - Joshua Winters-Brown

#### License
GNU GPLv3

Developers that use the GNU GPL protect your rights with two steps:
(1) assert copyright on the software, and (2) offer you this License
giving you legal permission to copy, distribute and/or modify it.
