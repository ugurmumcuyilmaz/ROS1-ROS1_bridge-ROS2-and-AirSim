# ROS1-ROS1_bridge-ROS2-and-AirSim
# ugurmumcuyilmaz.com

Controlling Vehicles with ROS1_bridge and ROS2 on Ubuntu 20.04 - AirSim Simulation Environment.

First, we start with 3 files. The names of the files in the order we'll run them are:

- airsim_controller.py
- control_command_converter.py
- user_input_publisher.py

Now, let me explain how to run them.

Firstly, the operating system I use is Ubuntu 20.04, and I have installed both ROS1 and ROS2. Additionally, I've installed ROS1 Bridge. This is because we can run AirSim with ROS1 for simulation, but in our project, we want to use ROS2 and AirSim. Below are the installed systems:

- Ubuntu Linux - Focal Fossa (20.04)
- ROS Noetic Ninjemys
- ROS 2 Foxy Fitzro
- ROS1 Bridge
- AirSim
- Unreal Engine 4.27

First, we need to open AirSim. You can open its interface by entering the necessary code in the terminal. After these systems are ready, open a terminal inside the ROS2 workspace (ROS2_ws). Then, first run the airsim_controller.py file. Next, open another terminal, also within the workspace. In the new terminal, run the control_command_converter.py file. Then open one more terminal, also within the workspace. In that terminal, run the user_input_publisher.py file. Afterwards, you'll be able to control the vehicle in the AirSim environment that opens up.

Feel free to refer to the documentation for guidance if you get stuck. --> https://docs.ros.org/

I'll also provide instructions on how to set up the system from scratch with code soon. For now, that's it.

You can see how it works in the video.

https://www.youtube.com/watch?v=667v1vihbQg
