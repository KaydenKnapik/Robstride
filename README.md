# Introduction
After working with the Robstride motors for quite some time now, I determined it would be best to make it easier for everyone to use. This git combines knowledge from Brian Smith's already existing Rostride Python SDK as well as from Feather Robotics Inc. Feather Robotics (https://feather.dev/) develops and manufactures general-purpose robots that are viable for entrepreneurs to build a company around.

# Needed Hardware
In order to make use of this repo, one would need access to a Robstride Motor (O1, O2, O3 or O4), and a CAN transciever capable of utilizing socketcan.

Robstride Motor Link: https://robstride.com/

Recommended Debugger Purchase Link (Choose Isolated Version): https://a.aliexpress.com/_mMUzKlH

Power supply (I am using 40V battery)

# 1. Hardware Setup
![IMG_0260](https://github.com/user-attachments/assets/c9809cf0-6164-491e-9297-b52acd4c3f37)


# 2. Code Setup 
Create a 'virtual enviornment'

```python3.10 -m venv Robstride```

Activate the Virtual Enviornment

 ```source Robstride/bin/activate```

Install Brian Smith's SDK:

The simplest way to install is via `pip`:

```python3 -m pip install robstride```

Then to setup the can controller run: (This must be run everytime reconnecting to the motor)

 ```sudo ip link set can0 type can bitrate 1000000```
 
 Then
 
 ```sudo ip link set can0 up```


# Programming the Motor
I have attached some basic code to this repo. In this repo you will find a 

```findangle.py```

This code allows you to find the motor angles that you may want your robot joint to rotate to.

```Rotateangles.py```

This code allows you to rotate one or more motors to set positions one after another.

```simultaneous.py```

This last code example allows you to move multiple motors at the same time.
# Questions? 
If you have any questions whatsoever feel free to mention me in the robstride discord or reach out to me by email at ailuroxrobotics@gmail.com


