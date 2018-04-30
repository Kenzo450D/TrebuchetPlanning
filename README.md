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

```
cd simTests
python kinSim2.py simpleWorldKobuki.xml
```

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
