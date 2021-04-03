# Particle Filter
Udacity Self-Driving Car Engineer Nanodegree Program

This project involves the a Simulator developed by Udacity which can be downloaded [here](https://github.com/udacity/self-driving-car-sim/releases).

## Dependencies

* **g++ 9.3** Installation Instructions: [Mac](https://developer.apple.com/xcode/features/) [Windows](http://www.mingw.org/) 
* **Ubuntu Terminal** for running UNIX Terminal on Windows. [download](https://aka.ms/wslubuntu2004)
* **uWebSocketIO** for smooth data flow between simulator and code. [download](https://github.com/uWebSockets/uWebSockets)  
    ```
    <Navigate to the project folder using Ubuntu terminal>
    chmod u+x install-ubuntu.sh
    ./install-ubuntu.sh
    ./install-linux.sh
    ```
* **cmake 3.5** [Installation Instructions](https://cmake.org/install/)  
* **make 4.1 (Linux and Mac), 3.81 (Windows)**  Installation Instructions : [Mac](https://developer.apple.com/xcode/features/) [Windows](http://gnuwin32.sourceforge.net/packages/make.html)

## Installation and Usage

This repository includes two files that can be used to set up and install uWebSocketIO for either Linux or Mac systems. For windows you can use either Docker, VMware, or even Windows 10 Bash on Ubuntu to install uWebSocketIO.

Once the install for uWebSocketIO is complete, the main program can be built and ran by doing the following from the project top directory.

    mkdir build
    cd build
    cmake ..
    make
    ./particle_filter

Alternatively some scripts have been included to streamline this process, these can be leveraged by executing the following in the top directory of the project:

    ./clean.sh
    ./build.sh
    ./run.sh

Here is the main protocol that main.cpp uses for uWebSocketIO in communicating with the simulator.

INPUT: values provided by the simulator to the C++ program
```
// sense noisy position data from the simulator

["sense_x"]

["sense_y"]

["sense_theta"]

// get the previous velocity and yaw rate to predict the particle's transitioned state

["previous_velocity"]

["previous_yawrate"]

// receive noisy observation data from the simulator, in a respective list of x/y values

["sense_observations_x"]

["sense_observations_y"]
```

OUTPUT: values provided by the c++ program to the simulator

```
// best particle values used for calculating the error evaluation

["best_particle_x"]

["best_particle_y"]

["best_particle_theta"]

//Optional message data used for debugging particle's sensing and associations

// for respective (x,y) sensed positions ID label

["best_particle_associations"]

// for respective (x,y) sensed positions

["best_particle_sense_x"] <= list of sensed x positions

["best_particle_sense_y"] <= list of sensed y positions
```

## Data

The data is located in a file titled *map_data.txt* under the directory titled *data*.

### Format:

Each row has three columns:
1. x position
2. y position
3. landmark id

The directory structure of this repository is as follows:

```
root
|   build.sh
|   clean.sh
|   CMakeLists.txt
|   README.md
|   run.sh
|
|___data
|   |   
|   |   map_data.txt
|   
|   
|___src
    |   helper_functions.h
    |   main.cpp
    |   map.h
    |   particle_filter.cpp
    |   particle_filter.h
```

## Output

The image below shows three points at each timeframe. There are landmarks (marked with crossed black circles) but only some are within range of the sensors in the car (marked with green lines). Using these landmarks, the car estimates its position on the map (marked with blue circle). We know the Particle Filter works because it more or less coincides with the position of the car.

[image1]: ./pf.PNG "Working PF"
![alt text][image1]
