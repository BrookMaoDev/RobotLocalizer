# LEGO EV3 Robot Localization Algorithm

https://github.com/user-attachments/assets/75448525-e5dd-46a3-bac1-8ea8221532f0

## What It Does

Give the program controlling the robot a map of its "terrain" and a destination to go to. Despite not knowing where it is on the map, the robot will scan its surroundings until it gains confidence about where it is, and then head towards the destination.

## How It Works

The program uses the Robot Localization algorithm discussed in [these notes](https://www.cs.utoronto.ca/~strider/docs/C85_Localization.pdf).

It also uses denoising techniques, like white balancing and averaging color sensor readings, to improve sensor accuracy. To classify colors, it converts RGB readings to HSV cylindrical coordinates and compares the distances between the read coordinates and the coordinates of the calibrated colors.

## Compiling and Running

Compile by running:

```bash
./compile.sh
```

Then run the executable with a map and a destination:

```bash
./a.out map.ppm <dest_x> <dest_y>
```

The executable will run a simulation of the robot localization algorithm, where you have to envision where the robot is and enter what it would scan. The algorithm will then print out where it thinks it is and the actions it will take in order to head towards the target.

## Running on a Real Robot

To run it on an actual robot, comment out `#define SIMULATE` in `EV3_Localization.c`. However, it probably will not work on any robot other than our own, since the code depends on how the robot is built.
