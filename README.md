# 16-350 Planning Techniques for Robotics Final Project

This is my code for the 16-350 Final Project.

## Code Instructions
```
cd ~/Documents
mkdir <name>_ws
cd <name>_ws
mkdir src
cd src
git clone https://github.com/svelado/underwater_path_planning.git
cd ..
catkin_make
source devel/setup.bash
roslaunch underwater_path_planning path_planning.launch
```

This launches rviz as well as the main server. In order to send desired start position and goal position for a path open a new terminal

```
cd ~/Documents/<name>_ws
source devel/setup.bash
rosrun underwater_path_planning plan_path_client <startX> <startY> <goalX> <goalY>
```
### Supplementary Comments
Gradescope has been giving me issues since the beginning of the year. In case something does not get uploaded or something, you can go to this link to get everything https://svelado.github.io/path_planning_project.html . It should have all the code links and report as well as a website to present the project. So in case gradescop fails I set this up so I have a submission ready at all times, as well as adding the project to my resume book. 
