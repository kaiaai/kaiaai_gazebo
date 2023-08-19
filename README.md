# [Kaia.ai](https://kaia.ai) pet robot simulation

[Kaia.ai](https://kaiaai) is a platform for 3D-printable pet robots. Please sign up for an early launch invite [here](https://remake.ai).

This repo is a ROS2 simulations meta-package for [Kaia.ai](https://kaia.ai) pet robots.

## Your PC setup
- If you are using a Windows PC, install [Windows WSL 2](https://learn.microsoft.com/en-us/windows/wsl/install)
and [Docker Desktop for Windows](https://docs.docker.com/desktop/install/windows-install/)
- When running in Docker for Windows, also install [XLaunch](https://sourceforge.net/projects/xming/)
to display GUI from the container - Rviz2, Gazebo, rqt, etc. Launch XLaunch and set its
*display number to zero* when prompted.
- When using a Linux PC, install [Docker Engine](https://docs.docker.com/engine/install/ubuntu/) or
[Docker Desktop](https://docs.docker.com/desktop/install/linux-install/) (with GUI)

## Launch the development Docker image
Open a Linux or Windows shell and run:
```
docker run --name kaia-ros-dev-humble -it -p 8888:8888/udp -e DISPLAY=host.docker.internal:0.0 -e LIBGL_ALWAYS_INDIRECT=0 kaiaai/kaia-ros-dev:humble
```
This command above launches the Kaia.ai developer Docker image and gives you bash prompt.

## Simulate the default robot model
- Run each command below in a separate terminal window
- Optionaly, set the world to be launched using the `world` argument,
e.g. `world:=empty_world.world`. `kaia_world.world` launches by default
- Keep in mind that launching the Gazebo simulator for the very first time can take a minute
or two - please be patient
```
ros2 launch kaia_gazebo world.launch.py description:=kaia_snoopy_description
ros2 run kaia_teleop teleop_keyboard
ros2 launch kaia_gazebo self_drive_gazebo.launch.py description:=kaia_snoopy_description
ros2 launch kaia_bringup rviz2.launch.py description:=kaia_snoopy_description
```
- `world.launch.py` launches Gazebo simulator populated with a world an instance of your bot
- `teleop_keyboard` lets you drive the bot manually
- `self_drive_gazebo` makes the bot self-drive around in Gazebo simulations
- `rviz2.launch.py` launches Rviz viewer. You will need Rviz viewer for navigation (see below)
to manually set the bot's initial position estimate as well as specify navigation goals,
i.e. where you want your bot to move
- if `description:=kaia_snoopy_description` is omitted, the value of `KAIA_ROBOT_DESCRIPTION`
variable set in your `.bashrc` is used. If the variable is not set, `kaia_snoopy_description` is used by default.

Press CTRL-C one or more times in each terminal window to stop the simulation.

To open a new terminal window, launch a new a Linux or Windows shell (outside Docker) and run:
```
docker exec -it kaia-ros-dev-humble bash
```

## Simulate a different robot model
- Select the robot model you would like to simulate by setting `description` to the robot description
package name, e.g. `description:=r2d2_description`. This is useful if you are modding an
existing robot model
- If the `description` argument is omitted, the value of `KAIA_ROBOT_DESCRIPTION` environment
variable will be used. If that environment variable is not set, `description` will default
to `kaia_snoopy_description`
```
ros2 launch kaia_gazebo world.launch.py description:=r2d2_description
ros2 run kaia_teleop teleop_keyboard
ros2 launch kaia_gazebo self_drive_gazebo.launch.py description:=r2d2_description
ros2 launch kaia_bringup rviz2.launch.py description:=r2d2_description
```

## Run SLAM, generate a map
Run each command below in a separate terminal window.
```
ros2 launch kaia_gazebo world.launch.py
ros2 launch kaia_cartographer cartographer.launch.py use_sim_time:=True
ros2 launch kaia_gazebo self_drive_gazebo.launch.py
ros2 run nav2_map_server map_saver_cli -f $HOME/my_map
```
- `cartographer.launch.py` launches the SLAM package and starts generating a map. You can see the map
gradually appearing in Rviz viewer.
- run `map_saver_cli` to save the map after the bot has driven around long enough to thoroughly map the world.

Press CTRL-C one or more times in each terminal window to stop the simulation.

## Navigate to a goal
```
ros2 launch kaia_gazebo world.launch.py
ros2 launch kaia_navigation nav2.launch.py use_sim_time:=True map:=$HOME/my_map.yaml
```
- `nav2.launch.py` launches the navigation package and loads the map you created in the previous step
- Before your bot can navigate, i.e. self-drive itself, to a destination of your choice, you must
manually specify the approximate initial location of your bot.
    - Click the `2D Pose Estimate` button on the upper toolbar in Rviz
    - Click on the map at the location where your bot currently is. Hold your mouse button down and drag your mouse in the direction your bot is facing. Now you can release your mouse button. Rviz should show your bot's location on the map.
- Specify where you would like your bot to navigate, i.e. the goal location
    - Click the `Nav2 Goal` button on the upper toolbar in Rviz
    - Clock on the map at the navigation goal location. Hold your mouse button down and drag your mouse in the direction you want your bot to face once it arrives to its goal location. Now you can release your mouse button. Rviz should display your bot's planned path and your bot should start moving.

Press CTRL-C one or more times in each terminal window to stop the simulation.

## Acknowledgements
Initial versions of packages in this repo are based on ROBOTIS [Turtlebot3 code](https://github.com/ROBOTIS-GIT/turtlebot3_simulations)
