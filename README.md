# Omnidirectional robot control based on vision - CPR - 4º GIERM

## Packages
- **camera_description** : camera model for gazebo
- **nexus_description** : high level description of our robot model
- **omnirobot_control** : robot and trajectory control
- **percepcion** : mapping and robot localization
- **planificador** : path planning

## How to launch the project
- Compile all the packages in your catkin workspace.

- Execute the full project:

    > roslaunch proyecto_gazebo proyecto.launch

- The window titled *"Seleccion_punto_final"* allow to select the final desired robot positition clicking on the map.

    - Another window appear showing us the trayectory planned.

    - The robot will start to move following it.

    - You can click again on the previous window and the system will replan a trajectory for the new selected point.

    - You can add obstacles to the world intersecting the current trajectory, and a new trajectory will be replanned to avoid that.

- The desired orientation of the robot can be changed with:
    > rosparam set /angle_robot [new_value]

## Demo
Click [here](https://youtu.be/VeuB0tvxSow) to see the demo video.

![Gazebo image](https://github.com/davidtr99/OmniRobot_CPR/blob/main/preview.jpg?raw=true)


## Authors
- David Tejero Ruiz
- José Manuel González Marín
- Miguel Granero Ramos
- Maria Castro Soto
- Pablo Díaz Rodríguez
