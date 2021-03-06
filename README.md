# Robot Motion Planning for a Trebuchet and Foot soldier configuration

The motivation of my proposed problem comes from the soldiers of medieval times protecting a packed trebuchet. Trebuchets are made of wood, and are vulnerable to ambush attacks.A packed trebuchet while being transported, is protected by a formation of foot soldiers or cavalry walking around the packed trebuchet until its unpacked. The soldiers encircle the trebuchet in a circle or square formation, so that any attack cannot penetrate to the trebuchet. The formation must be evenly distributed so that the foot soldiers do not bump into one another while making turns. Moreover, they should be able to traverse through tight spaces by contracting and expanding their size by changing the inter-foot-soldier distance. All manuevers of the soldiers should be made keeping sure that no side of the trebuchet goes unprotected.

The trebuchet is modeled as a differential drive robot which can make point turns. Although this is not historically accurate, I made it for the simplicity. The trebuchet is represented as the turtlebot. The footsoldiers are represented by spheres. These spheres DO NOT have a no-slip constraint, thereby allowing them to have holonomic motion across the workspace.

## Implementation details

The path planning is done through Bi-directional RRT, with a motion constraint of turns limited to pi/18. Although this is  violated in case of when the two RRT paths meet. The spherical robots need to be around the turtlebot within a certain distance threshold.

I will only implement a case of 3 spherical holonomic robots. The path planning is done by a larger composite robot making the path planning. Depending on the positions generated by this composite robot path for the RRT path, the smaller turtlebot and the spheres can move.

## Getting Started

These instructions will get you a copy of the project up and running on your local machine for development and testing purposes.

Python2 libraries
Install Klampt with python bindings.
Run the code with these following commands:

To run the single robot RRT:
```
cd simTests
python kinSim2.py simpleWorldKobuki.xml
```

The multi robot RRT just uses a convex hull to approximate the size of the formation. As it is not 100% implemented correctly, the output does has a visualization of only the convex hull and not of the individual robots.
To run the multi robot RRT:
```
cd simTests
python kinSimMulti.py simpleWorldMultiRobot.xml
```

To run the complete Demo with a turtlebot and 4 spheres representing people walking around it, execute this:

```
cd simTests
python kinSimMultiAgent.py simpleWorldMultiAgentSystem.xml
```

### Results

The following images illustrate the single robot RRT path from start to goal position:

![Single Robot RRT Image1](docs/images/pic1.png?raw=true "Single Robot RRT - #1")
Figure 1: Single Robot RRT (Image 1/9)
![Single Robot RRT Image1](docs/images/pic2.png?raw=true "Single Robot RRT - #2")
Figure 2: Single Robot RRT (Image 2/9)
![Single Robot RRT Image1](docs/images/pic3.png?raw=true "Single Robot RRT - #3")
Figure 3: Single Robot RRT (Image 3/9)
![Single Robot RRT Image1](docs/images/pic4.png?raw=true "Single Robot RRT - #4")
Figure 4: Single Robot RRT (Image 4/9)
![Single Robot RRT Image1](docs/images/pic5.png?raw=true "Single Robot RRT - #5")
Figure 5: Single Robot RRT (Image 5/9)
![Single Robot RRT Image1](docs/images/pic6.png?raw=true "Single Robot RRT - #6")
Figure 6: Single Robot RRT (Image 6/9)
![Single Robot RRT Image1](docs/images/pic7.png?raw=true "Single Robot RRT - #7")
Figure 7: Single Robot RRT (Image 7/9)
![Single Robot RRT Image1](docs/images/pic8.png?raw=true "Single Robot RRT - #8")
Figure 8: Single Robot RRT (Image 8/9)
![Single Robot RRT Image1](docs/images/pic9.png?raw=true "Single Robot RRT - #9")
Figure 9: Single Robot RRT (Image 9/9)

### Built With

* [Klampt](http://motion.pratt.duke.edu/klampt/tutorial_install.html) - Robot Visualization Software


## Authors

* **Sayantan Datta** 

* **Saurav Agarwal** - *Initial work* - [klampt simulation](https://github.com/AgarwalSaurav/klampt_simulations)

## License

This project is licensed under the GNU GENERAL PUBLIC LICENSE - see the [LICENSE](LICENSE) file for details

## Acknowledgments

* [Saurav Agarwal's](https://github.com/AgarwalSaurav) code and tutorial on Klampt Simulation
* [Steven M. LaValle](http://msl.cs.illinois.edu/~lavalle/index.html) code on basic [RRT](msl.cs.illinois.edu/~lavalle/sub/rrt.py)
* The project was inspired from Age of Empires2, Microsoft Studios
