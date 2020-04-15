# local planner

This repository is dedicated to collecting the latest planners which are developed in ROS.

## Clone this project

```
git clone --recursive git@github.com:yangmingustb/Local-Planner.git
```

## Dependencies
* C++11 standard, use Rviz to show algorithm, so you should install [ROS](https://www.ros.org/).
* map_server
* EIGEN
* ompl
* FCL
* ...(wait to test)

I will write an installation srcipt to install these requirements.

run the script
```
install_all_dependencies.sh
```

## how to use
1. cd Local-Planner
2. catkin_make
3. source devel/setup.bash
4. launch script, e.g.  
```roslaunch astar_planner a_star_planner.launch```

## algorithms
### astar_planner
- to do: update display errors in rviz
### hybrid_astar_planner_hkust
- todo: 提取hybrid a star planner
### rrt_star_planner
- to do
### hybrid_astar_planner_kth
- have done!
### simple_planner_repo

### cubic_polynomial_planner

### prm_planner
- to do
### spiral_planner 
- to do

### fast_planner
- from hkust
  
### reference_line  

### state_lattice_planner
- to do

### frenet_lattice_planner  
- to do
### rrt_connect_planner 
- to do
### theta_star_planner
- to do
### hybrid_astar_planner_apollo
- to do: 需要提取
### rrt_planner
- to do




