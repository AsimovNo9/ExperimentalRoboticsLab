<a href="https://unige.it/en/">
<img src="images/genoa_logo.png" width="20%" height="20%" title="University of Genoa" alt="University of Genoa" >
</a>

# Experimental Robotics Lab Project 1

>**Author: Adedamola Ayodeji Sode**   
 **Email: adedamola.sode@gmail.com** </br>
 **Student ID: S5360004**

 # Outline
 * Brief Desscription
 * Installation Guide
 * Software Architecture Description
 * Intended Implementations

 # Brief Description
  This project is based off the popular board game cluedo. Using the game's simulation of a murder mystery with intents at deriving at guesses to attain important hints at arriving at the final conclusion, *who* killed, *where* did the kill take place, and *what* was the murder weapon.

  
 # Installation
 1. Clone the git repo into your ROS workspace
```bash
git clone https://github.com/AsimovNo9/ExperimentalRoboticsLab.git
```
 2. Rename the folder into "experimental_robotics" as that is how the package is structured to remove the descripancies of the repo

 3. Build the workspace
 ```bash
catkin_make
```
 4. Source the Setup.bash if not already done
 ```bash
source devel/setup.bash
```
 5. Launch the package
```bash
roslaunch experimental_robotics CluedoLaunch.launch
```

 # Software Architecture Description

<img src="/images/State-Image.jpg" alt="State Machine" width="" height="">

Given the following state machine, we will design an architecture which centers the state machine as the core of the of the entire package. Where the state machine inquires as a client to the navigation service node, the oracle service node, the knowledge service node and the gather hints service node. These in turn return the state machine with its required responses so that the process is continued.

<img src="/images/ComponentDiagram.jfif" alt="Component Diagram" width="" height="">

# Intended Implementations

The project is intended to be bolstered given these set of features:



|Priority|            Feature               |                               Description                                      |      Status      |
|:------:|:--------------------------------:|:------------------------------------------------------------------------------:|:----------------:|
|    1   |              Armor               |           Implementing Armor client service for ontology inclusion             |   :white_check_mark:  |
|    2   |             ROS Plan             |                  Implementing ROS Plan to describe                             |  :construction:  |
|    3   |        A Navigation Stack        | Replacing the random naviagtion implementation with a proper navigation stack  |    :bookmark:    |
|    4   |  Robot Modeling and Simulation   |       URDF and CAD modelling of robot, with move_it implementation             |    :bookmark:    |
  


 
