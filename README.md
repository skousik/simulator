# simulator2D

This is a set of tools for simulating a planar vehicle's dynamics in MATLAB. The three key components for this simulator are:

1) Agent: the vehicle that will be simulated. See agent2D.m for an overview.

2) Planner/s: the planning algorithm/s that are used to conduct the agent through a simulated environment. See planner2D.m for the overview.

3) World: the environment that the agent must navigate through. This can be static, dynamic, and as big or small as needed. So far, we only have a small room full of boxes available. See world2D.m for an overview.

## Getting Started

Make sure simulator2D and all subfolders are on your path, and you're ready to go!
Currently, there is no example simulation in this repository, but you can find examples here:

https://github.com/skousik/FRS_trajectory_planning/tree/master/examples

The examples are, in particular, for a Segway RMP robotic platform that we have in ROAHM Lab at the University of Michigan. To run the simulation examples, the FRS planner will require you to download the FRS_trajectory_planning repository, and the GPOPS planner will require you to have GPOPS installed. However, the RRT planner is self-contained, so go ahead and run the "run_segway_RRT_simulation.m" file with FRS_trajectory_planning on your path and see what happens!

### Prerequisites

To understand why all this exists, you should read this paper: https://arxiv.org/abs/1705.00091

We've created this repository as a way to quickly test our trajectory planner, and to evaluate it against the state of the art. This repository will grow quickly as we publish more papers.

## Authors

Shreyas Kousik (https://github.com/skousik)

Sean Vaskov (https://github.com/skvaskov)

## License

This project is licensed under the MIT License.

