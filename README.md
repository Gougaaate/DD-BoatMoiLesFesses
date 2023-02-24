# DD-BoatMoiLesFesses
## Project Description
We had to program the guidance of a boat using python. Here, you can find all the .py modules which enable us to guide the boat.
In the drivers folder, all the programs that control Arduino and sensors are presented
## General Information

### Project Status
100% all tasks done
### Work in Progress
Last work done : following a circle using a Kalman filter
### Work Done
* Calibration of the compass.
* Following of a heading, for example 70° from the starting point.
* Following a triangle using linearized GPS coordinates.
* Following a circle using feedback linearization and Kalman filter.
## Dependencies
You need the drivers folder, and the roblib.py file.
## How to use the code ?
You have to use the terminal to bind yourself to the boat using a ssh protocol : 
```console
$ ssh ue32@172.20.25.205
$ ue32
$ cd ddboatmoilesfesses
$ python3 whatever_you_want.py
```
  The file *heading_command.py* enables you to follow a gps coordinate, you can just run *main.py*  to execute it.