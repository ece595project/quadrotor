---
title: Quadrotor SAR Project
layout: default
---

# {{page.title}}

{% include youtubePlayer.html id="PK7pUsQQ6ss" %}

## Table of Contents

* This will become a table of contents (this text will be scraped).
{:toc}

## Project Members

- [Jacob Jizewski](https://github.com/jacoblizewski) (Primary ROS Coder)
- [Bryan Williams](https://github.com/bwilliams44) (Device & Module Testing)
- [Adam Thorpe](https://github.com/ajthor) (Documentation & Coding)

## Purpose

Search and rescue is a dangerous job, requiring a coordinated, large-scale effort to locate survivors of an accident or disaster. Rubble is often unstable, and wilderness areas can be too large to search effectively in a limited amount of time. Our proposed project is to create a swarm control system for aerial (and/or ground-based) robots for search and rescue tasks. We've chosen quadrotors as our target robots due to their ability to traverse a wide area quickly and their maneuverability in order to compensate for obstacles. A large block may be too large for a ground-based robot to maneuver over, but an aerial robot can maneuver in almost any known space.

The big-picture implementation of our project would be for search and rescue assistance. Aerial robots could conduct a coordinated survey of a large area, giving rescue personnel critical information to safely navigate to found victims. However, with the time that we have to complete our project, coordinated "swarm" movement will be our core focus.

## Milestones

- [x] Worked with an actual quadrotor to learn how to subscribe to ROS topics from actual sensor data.
- [x] Implement ROS code for quadrotor movement.
- [x] Test component integration with ROS topics.
- [x] Create search algorithm.
- [x] Integrate search algorithm into ROS code.
- [x]

## Initial Proposal

Take a look at our [initial proposal](https://ece595project.github.io/quadrotor/Initial-Proposal) for the project, and then view the relevant sections below to see how things have changed since the start of the project.

- [Cost](https://ece595project.github.io/quadrotor/Cost)
- [Difficulties](https://ece595project.github.io/quadrotor/Difficulties)
- [Measurement](https://ece595project.github.io/quadrotor/Measurement)
- [Organization](https://ece595project.github.io/quadrotor/Organization)
- [Software](https://ece595project.github.io/quadrotor/Software)

## Links

Below you will find links to the project pages, detailing our design project.

### Design Documents

- In the [development guidelines](https://ece595project.github.io/quadrotor/Development), you will find information on how we developed our project using [docker](https://www.docker.com). In the [deployment instructions](https://ece595project.github.io/quadrotor/Deployment-Instructions), you will find information on how to set up ROS to run our demo.

- On the [search algorithm](https://ece595project.github.io/quadrotor/Search-Algorithm) page, you will find information on how we designed our search and rescue algorithm.

- On the [docker](https://ece595project.github.io/quadrotor/Docker) page, you will find information on our design methodology to utilize Docker in our swarm.

- On the [dynamics](https://ece595project.github.io/quadrotor/Dynamics) page, you will find information on the resources we used to define the dynamics of the quadrotor.

- In the [quadrotor BOM](https://ece595project.github.io/quadrotor/Quadrotor-BOM), you will find information on how we plan to source the components of our quadrotors for physical testing.

### Testing Documents

- In the [testing](https://ece595project.github.io/quadrotor/Testing) document, you will find information on how we tested our algorithm and information on our experience with real hardware running ROS.

### Reports

- We have included a [project report](https://ece595project.github.io/quadrotor/Project-Report), summing up our experiences working on this project and how we would like to proceed in the future.


**[back to top](#table-of-contents)**
