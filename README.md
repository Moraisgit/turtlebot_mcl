# turtlebot_mcl

**A Monte Carlo Localization (MCL) algorithm developed by students for the Autonomous Systems (SAut) subject lectured at Instituto Superior Técnico (IST).**

## Menu

  - [**Prerequisites**](#prerequisites)
  - [**Install**](#install)
  - [**Source Code**](#source-code)
  - [**Launch Files**](#launch-files)
  - [**Scripts**](#scripts)
  - [**Wiki Page**](#wiki-page)

## Prerequisites
- [Ubuntu](https://ubuntu.com/download) (tested with 18.04 and 20.04)
- [ROS](http://wiki.ros.org/ROS/Installation) (tested with Melodic and Noetic)

## Install
Use the following commands to download and compile the package. Change `catkin_ws` depending on your setup.

```
$ cd ~/catkin_ws/src
$ git clone https://github.com/Moraisgit/turtlebot_mcl_localization.git
$ cd ..
$ catkin_make
```

## Source Code
The functionalities of each script are below described.
- `busca.py` - Implements the algorithm for real data (and for Gazebo data).

- `graphs1.py` - Implements a graph (to be ran with `busca.py`) for the robot's position and the particles.

- `graphs2.py` - Implements a graph (to be ran with `busca.py`) showing the robot's path estimated by our code (weighted average with 1% of the heaviest particles) along with the path followed by [AMCL](https://wiki.ros.org/amcl). It also displays the RMSE in another graph.

- `Microsimulador_MCL.py` - Implements the algorithm for synthetic data.

- `MSgraphs1.py` - Implements a graph (to be ran with `Microsimulador_MCL.py`) for the robot's position and the particles.

- `MSgraphs2.py` - Implements a graph (to be ran with `Microsimulador_MCL.py`) showing the robot's path estimated by our code (weighted average with 1% of the heaviest particles) along with the actual path taken by the robot (known <i>a priori</i>, not using [AMCL](https://wiki.ros.org/amcl)). It also displays the RMSE in another graph.

## Launch Files
There are two lauch files that can be used.
- `busca.launch` - Launches the scripts `busca.py`, `graphs1.py` and `graphs2.py` to ease the process of visualizing real data and Gazebo data (in order for the graphs regarding [AMCL](https://wiki.ros.org/amcl) data, launch the method itself).

- `micro_simulator.launch` - Launches the scripts `Microsimulador_MCL.py`, `MSgraphs1.py` and `MSgraphs2.py` to ease the process of visualizing microsimulator data.

## Scripts
Three scripts were made in order to perform metric analysis in our project.
- `correct_time_csv.py` - Removes decimal values of the timestamps in `.csv` files in order to accuratly plot several files of data synchronized.

- `rmse_max_min.py` - Determines the maximum and minimum value of RMSE in a `.csv` file before and after kidnapping occurs.

- `rmse_mean_SD_SEM.py` - Calculates the mean of sevreal RMSE tests, standard deviation and standard error of the mean.

## Wiki Page
Visit our Wiki Page [here](https://github.com/Moraisgit/turtlebot_mcl/wiki/Wiki-page-of-turtlebot_mcl_localization) to see some important tools we used for this project.
