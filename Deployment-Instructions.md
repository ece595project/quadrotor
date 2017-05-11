---
title: Deployment Instructions
layout: default
description: Deployment instructions to start the Quadrotor SAR demo.
---

# {{page.title}}

{{page.description}}

[Home](https://ece595project.github.io/quadrotor/)

## Table of Contents

* This will become a table of contents (this text will be scraped).
{:toc}

## Setup

1. Install the Hector Quadrotor ROS package

1. Binary package

    ```Shell
    $ sudo apt-get install ros-indigo-hector-quadrotor
    ```

    If you want to look at indoor or outdoor demos with hector_quadrotor

    ```Shell
    $ sudo apt-get install ros-indigo-hector-quadrotor-demo
    ```

1. Running "swarm demo"

    1. Build the included `ece595project` package in your workspace.

    2. Run: `roslaunch ece595project swarm_empty_world.launch`
        Launch file will open the empty world gazebo model with three hector_quadrotor models.

    3. Run in separate terminal: `rosrun ece595project MasterHand`.
        Node with raise and lower all three quadrotor models in a sinusoidal pattern .

        Node based on MasterHand.cpp in src.
