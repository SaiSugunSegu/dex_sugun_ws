
# Assignment of Dexory - Sugun


## Instruction to build and run the Assignment
### Assuming ADE and docker installed 

- Current workspace can work with ADE image provided with assignment, so just replacing the dex_ws with dex_sugun_ws will allow to use the custome plugin.

Step 1: Setup home directory and build image
```
mkdir ~/dexoryhome
cd ~/dexoryhome
touch .adehome

cp <downloaded_exercise.zip> ~/dexoryhome
unzip exercise.zip

cd ~/dexoryhome/auto-exercise/ade
docker build --no-cache -t auto-exercise-ade:latest
```

Step 2: Start and Enter into Development Environment
```
cd ~/dexoryhome/auto-exercise
ade start --update
ade enter
```

Step 3: Replace the dex_ws to dex_sugun_ws
```
cd auto-exercise
rm -rf dex_ws
git clone https://github.com/SaiSugunSegu/dex_sugun_ws.git
cd dex_sugun_ws
colcon build --symlink-install
source install/setup.bash
ros2 launch dex_bringup tb3_simulation_launch.py
```

## Results:
### When you run the launch file, Rviz window will be poped up, with Turtlebot Localized in Gazebo world map.

- Give a Nav2_goal pose.
- Turtlebot should be able to reach Goal pose with given features mentioned in Assignment.

<p align="center">
  <img src="https://github.com/SaiSugunSegu/dex_sugun_ws/assets/50354583/6a45b210-3cb2-4975-935f-3f561110215d">
</p>

_Note: Please find more about algorithm and implementation in readme.md_
