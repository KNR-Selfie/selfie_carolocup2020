# Selfie-autonomous-car
![chrupek](https://user-images.githubusercontent.com/28540485/74652634-bdccbb00-5186-11ea-8b1c-0661c07a055a.jpg)

*Selfie* is the student project of autonomous cars. Vehicles based on 1:10 scale RC cars are customized to be able
to operate autonomously in simulated road environments. They are equipped with camera, computer vision computing unit,
controller and set of sensors like magnetic encoders, distance sensors and IMU.

# Selfie Carolo-Cup2020

This repository contains the [catkin workspace](http://wiki.ros.org/catkin/workspaces) for Selfie Autonomous Car
at [Carolo-Cup 2020](https://wiki.ifr.ing.tu-bs.de/carolocup/news) competition.

## Build instructions

The project is targetting [ROS Kinetic](http://wiki.ros.org/kinetic) distribution. Make
sure you have it [installed](http://wiki.ros.org/kinetic/Installation) on your development machine.

Clone the repository to a convenient location using:

```bash
git clone --recurse-submodules https://github.com/KNR-Selfie/selfie_carolocup2020
```

Navigate to the main directory with:

```bash
cd selfie_carolocup2020
```

The following set of commands will in turn download all external dependencies, build the packages in
`src` directory and include them in your environment.

```bash
./resolve-dependencies.sh
catkin_make
source ./devel/setup.bash
