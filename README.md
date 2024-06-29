
# Spot Micro

Spot Micro Quadruped Dog Robot is a small, four-legged robotic dog designed to mimic real canine movements. It serves various purposes, including research, education, and entertainment. Named "Spot Micro" due to its smaller size compared to Boston Dynamics' Spot robot, it's customizable for fun projects or serious applications like exploring rough terrain and search and rescue operations. Spot Micro showcases advancements in robotics with wide-ranging potential uses.


## Motivation


As students in the Spot Micro community, we developed a reliable and versatile PyBullet simulator for safe experimentation. It also serves as a Gym environment for Reinforcement Learning (RL) tasks.

Even without using the Gym environment, the simulator is useful. The main.py script accepts any gait implementation and provides a GUI for testing. For example, we've implemented a 12-point Bezier gait.

This simulator is a valuable tool for students, offering a safe and flexible platform for testing and development.


## Features

- X-axis Motion
- Y-axis Motion
- Z-axis Motion
- Roll Motion
- Pitch Motion
- Yaw Motion
  
More coming soon.....

<img width="1280" alt="spot" src="https://github.com/Bug-Finderr/Spot_Micro/assets/147031450/ac3e2ccc-35c1-4e7e-99a8-e4f66d098614">


## Environment Setup

First you will need to create an environment.

    python -m venv <env-name>

To run this project, you will need to add the following to your environment

- Scipy
- Numpy
- PyBullet
- Gym
- Matplotlib
- Seaborn
- Setuptools

To install use command : 
    
    pip install <package-name>

### How To Run Simulation

After installing the required dependencies.

- Start the environment by python : 

         .\env\Scripts\activate

- Navigate to the src folder : 

        cd .\simulation\src\

- Run the main.py file : 

        python .\main.py


## Hardware Requirements

If you want to build the real world working model, here is the list of required parts:

**[Requirements](https://docs.google.com/spreadsheets/d/1mMlYKB9iXG_FOO65WEjEGRvvVUh6q3yLi6lr9TdSkBI/edit?gid=0#gid=0)**


## Credits

**Inspiration from :** [spot_mini_mini](https://github.com/OpenQuadruped/spot_mini_mini)

**Leg kinematics :** [Inverse kinematics](https://www.researchgate.net/publication/320307716_Inverse_Kinematic_Analysis_Of_A_Quadruped_Robot)

**Lie Algebra :** [Code Snippets](https://github.com/NxRLab/ModernRobotics)

**Beizer curves :** [Beizer curves](https://dspace.mit.edu/handle/1721.1/98270)

**Rotation :** [Rotation logic](http://www.inase.org/library/2014/santorini/bypaper/ROBCIRC/ROBCIRC-54.pdf)

**Gym and env with the help of  :** [Coumans, Erwin and Bai, Yunfei](https://github.com/bulletphysics/bullet3/blob/master/examples/pybullet/gym/pybullet_envs/bullet/)








