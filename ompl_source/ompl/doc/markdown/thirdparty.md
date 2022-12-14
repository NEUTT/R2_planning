# Contributions

This page includes a list of extensions to OMPL provided by the community. Please read the [directions on writing a contribution](contrib.html) and the suggested [style guide](styleGuide.html) if you would like to submit your own contribution.

# Generalized Optimal Path Planning Framework

### Author: Luis Torres (University of North Carolina at Chapel Hill)

Extend OMPL's support for optimal path planning. Users can plan robot paths that optimize a variety of ready-to-use metrics such as path length and clearance from obstacles. Users can also plan optimal paths under their own customized path metrics without changing OMPL's optimizing planners. Additional benchmarking data has also been added so users can efficiently track the progress of an optimizing planner on their planning problem. Tutorials for use of the framework can be found [here](optimalPlanning.html).

# LBT-RRT contribution

### Author: Oren Salzman (Tel Aviv University)

[LBT-RRT](\ref gLBTRRT) (Lower Bound Tree RRT) is a near-asymptotically optimal incremental sampling-based motion planning algorithm. LBT-RRT is guaranteed to converge to a solution that is within a constant factor of the optimal solution. The notion of optimality is with respect to the distance function defined on the state space we are operating on. The algorithm is described in:
- Oren Salzman and Dan Halperin, Asymptotically near-optimal RRT for fast, high-quality, motion planning,
_arXiv_, 2013 [[PDF]](http://arxiv.org/pdf/1308.0189.pdf)

# SPARS and SPARS2 contribution

### Author: Andrew Dobson (Rutgers University)

- [SPARS](\ref gSPARS) and [SPARS2](\ref gSPARStwo) are roadmap-based planners that operate similarly to Visbility-based PRM, but provide asymptotic near-optimality guarantees. The SPARS and SPARS2 algorithms a described in:
    - A. Dobson, A. Krontiris, K. Bekris, Sparse Roadmap Spanners, _Workshop on the Algorithmic Foundations of Robotics (WAFR)_ 2012. [[PDF]](http://www.cs.rutgers.edu/~kb572/pubs/sparse_roadmap_spanner.pdf)
    - A. Dobson, K. Bekris, Improving Sparse Roadmap Spanners, _IEEE International Conference on Robotics and Automation (ICRA)_ May 2013. [[PDF]](http://www.cs.rutgers.edu/~kb572/pubs/spars2.pdf)

# T-RRT Implementation

### Author: Dave Coleman (University of Colorado Boulder)

- [T-RRT](\ref gTRRT) is an RRT variant and tree-based motion planner that takes into consideration state costs to compute low-cost paths that follow valleys and saddle points of the configuration-space costmap. It uses transition tests from stochastic optimization methods to accept or reject new potential sates. [An example use of TRRT](https://github.com/davetcoleman/ompl_rviz_viewer).
    - L. Jaillet, J. Cort??s, T. Sim??on, Sampling-Based Path Planning on Configuration-Space Costmaps, in IEEE Transactions on Robotics, 26(4):635???646, August 2010, [[PDF]](http://homepages.laas.fr/nic/Papers/10TRO.pdf)

# ROS Interface to OMPL

### Authors: Ioan Sucan and Sachin Chitta (Willow Garage)

- [MoveIt!](http://moveit.ros.org) wraps OMPL as a planning plugin.

# RRT* and extensions

### Authors: Alejandro Perez and Sertac Karaman (MIT)

- [RRT*](\ref gRRTstar) (optimal [RRT](\ref gRRT)) is an asymptotically-optimal incremental sampling-based motion planning algorithm. RRT* algorithm is guaranteed to converge to an optimal solution, while its running time is guaranteed to be a constant factor of the running time of the RRT. The RRT* algorithm was introduced and analyzed in:

    - S. Karaman and E. Frazzoli, _Sampling-based algorithms for optimal motion planning_ Int. Journal of Robotics Research, 2011. Also available at http://arxiv.org/abs/1105.1186.

  For more information on RRT* and its variants see http://ares.lids.mit.edu/rrtstar.

# PRM extensions

### Author: James Marble (University of Nevada at Reno)

- Generalized the implementation of PRM so that different variations can be created by passing connection strategies and filters. Provided such functions for PRM and PRM*.


# RRT extension for planning with controls

### Author: Jennifer Barry (MIT)

- Generalized the implementation of RRT (with controls) so that intermediate states generated along motions are also optionally added to that tree of motions.
