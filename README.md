# MB3D-Printer
![](Records/FinalPrototype.jpg)
Random project that kept me busy during the 2020 summer quarantine.

Designed, assembled and programmed completely in-house.
It is an alternative type of 3d printer I thought of while considering the detrements of a motor moving along with a cartesian axis of traditional 3d printers. A malus already avoided in the CoreXY designs by not having any motors moving around, but I still wanted to build something innovative that could also retain the benefits of threaded rod instead of belt control. And (obviously) inspired by my recently finished highschool project ([DisegnatoreRemoto](https://github.com/bellimarco/DisegnatoreRemoto)) this polar axis control design was born.

## From 3D file to real object
Like for most 3d printer setups the first step is to design the object in some 3d modeling software (Fusion360) and exporting the related .stl file.
This 3d file must then be sliced through a slicer software (Ultimaker Cura) to generate the related .gcode file.

The printer itself is based on a ArduinoMega+RAMPS1.6 control unit, that manages 4 steppers, the hotend heater, 2 fans and 2 thermistors. It uses serial comunication with the computer to request Gcode that should be executed.

Once the Arduino is turned on and connected via USB, the node js application (mb3d.js) must be started from the computer terminal. This creates a server and makes available in the browser a rudimentary user interface to control the printer; through direct Gcode transmission to the board or indirect GUI interaction buttons like translate, home axis or prime extruder.

From the webpage the desired gcode file can be loaded into the application. Since the gcode file is generated by a traditional slicer for a traditional printer, it must first be "reprocessed" to be used (a process that involves primaraly transforming coordinates from cartesian to polar, but also other essential details and tunings for my specific design).

The print process can now begin. It goes on by sending the arduino packages of gcodes at a time, waiting that the printer executes them all and asks for the next package.

## Resources
In the Records folder there are many demo images of printed parts used to replace the old wooden ones

For some demo videos I refer you to my dedicated google drive [folder](https://drive.google.com/drive/folders/1-1SE9T9Bii7azsfO967qi_PxQ38gY363?usp=sharing)
