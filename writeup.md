# Project for the Localization course in Udacity's Self-Driving Car Nanodegree.


[//]: # (Image References)


[image1]: ./examples/Screenshot_20210602_181930.png
[image2]: ./examples/Screenshot_20210602_181949.png
[image3]: ./examples/Screenshot_20210602_181955.png



## Project Introduction
The robot has been kidnapped and transported to a new location! Luckily it has a map of this location, a (noisy) GPS estimate of its initial location, and lots of (noisy) sensor and control data.

In this project I implemented a 2 dimensional particle filter in C++. The particle filter will be given a map and some initial localization information (analogous to what a GPS would provide). At each time step the filter will also get observation and control data.

## Running the Code
This project involves the Term 2 Simulator which can be downloaded [here](https://github.com/udacity/self-driving-car-sim/releases)

This repository includes two files that can be used to set up and install uWebSocketIO for either Linux or Mac systems. For windows you can use either Docker, VMware, or even Windows 10 Bash on Ubuntu to install uWebSocketIO.

Once the install for uWebSocketIO is complete, the main program can be built and ran by doing the following from the project top directory.

1. mkdir build
2. cd build
3. cmake ..
4. make
5. ./particle_filter

Alternatively some scripts have been included to streamline this process, these can be leveraged by executing the following in the top directory of the project:

1. ./clean.sh
2. ./build.sh
3. ./run.sh

Tips for setting up your environment can be found [here](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/0949fca6-b379-42af-a919-ee50aa304e6a/lessons/f758c44c-5e40-4e01-93b5-1a82aa4e044f/concepts/23d376c7-0195-4276-bdf0-e02f1f3c665d)


# Implementation of the Particle Filter
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

The main file is `particle_filter.cpp` in the `src` directory. The file contains the `ParticleFilter` class and some associated methods.

The `src/main.cpp` file contains the code that will actually be running the particle filter and calling the associated methods.

## Inputs to the Particle Filter
You can find the inputs to the particle filter in the `data` directory.

#### The Map*
`map_data.txt` includes the position of landmarks (in meters) on an arbitrary Cartesian coordinate system. Each row has three columns
1. x position
2. y position
3. landmark id

All other data the simulator provides, such as observations and controls (Map data provided by 3D Mapping Solutions GmbH).

## Workflow of the particle filter

All the work is done in `particle_filter.cpp`. The structure is as follows:  
1. STEP: INITIALIZATION: Initialize all particles (x, y, theta) based on GPS input (+ Gaussian noise), set all weights to 1.0.  
`void ParticleFilter::init(double x, double y, double theta, double std[])`  
  
2. STEP: PREDICTION: Add the control input x, y and the yaw angle to all particles (+ Gaussian noise)  
`void ParticleFilter::prediction(double delta_t, double std_pos[], double velocity, double yaw_rate)`  
  
3. STEP: DATA ASSOCIATION WITH NEAREST NEIGHBOR TECHNIQUE: Find the predicted measurement that is closest to each observed measurement and assign the observed measurement to this particular landmark. 
`void ParticleFilter::dataAssociation(vector<LandmarkObs> predicted, vector<LandmarkObs>& observations)`  
  
4. STEP: UPDATE THE PARTICLES WEIGHTS:  
  1. Transform observations from vehicle's coordinates to map coordinates.  
  2. Filter on landmarks that are in range of the LiDAR.  
  3. Associate observations with predicted landmarks  
  4. Update the weights of each particle using a multvariate Gaussian distribution  
  5. Normalize the weights of all particles for the resampling step  
`void ParticleFilter::updateWeights(double sensor_range, double std_landmark[], const vector<LandmarkObs> &observations, const Map &map_landmarks)`
  
5. STEP: RESAMPLE PARTICLES WITH REPLACEMENT WITH PROBABILITY PROPORTIONAL TO THEIR WEIGHT: With use of Resampling Wheel technique  
`void ParticleFilter::resample()`

## Project Result

The implemented Particle Filter was run on Udacity's simulator and its error and performance was noted. Here are some screenshots of the simulator at work:  


![alt text][image1]

![alt text][image2]

![alt text][image3]