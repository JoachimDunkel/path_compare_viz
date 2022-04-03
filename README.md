## Overview

This repository tries to provide an easy way for comparing estimated 
and ground-truth paths taken by a robot in simulation.

It listens to `base_link_estimate` and `base_link_real` and publishes
nav_msgs/Path on both of their trajectories to display in rviz.

## Usage

Download or clone the repo.

Take a look inside `config.yaml` and change its properties according to your needs.

### Example

Real robot-path (Red)

Estimated path (Green)

<img src="example.png" title="" alt="" width="286">

*Note: This example was taking testing out my monte-carlo localisation implementation*