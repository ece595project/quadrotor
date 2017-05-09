# Quadrotor Search and Rescue

Team Members: Jacob Lizewski, Adam Thorpe, Bryan Wiliams

## Purpose

Search and rescue is a dangerous job, requiring a coordinated, large-scale effort to locate survivors of an accident or disaster. Rubble is often unstable, and wilderness areas can be too large to search effectively in a limited amount of time. Our proposed project is to create a swarm control system for aerial (and/or ground-based) robots for search and rescue tasks. We've chosen quadrotors as our target robots due to their ability to traverse a wide area quickly and their maneuverability in order to compensate for obstacles. A large block may be too large for a ground-based robot to maneuver over, but an aerial robot can maneuver in almost any known space. 

The big-picture implementation of our project would be for search and rescue assistance. Aerial robots could conduct a coordinated survey of a designated area, giving rescue personnel critical information to safely navigate to found victims. However, with the time that we have to complete our project, coordinated movement will be our core focus. 

## Difficulties

There are several complexities to the project we are proposing. Firstly, we will be creating our own quadrotors to use in this project. The reason for this is that there are simply not enough available to us in order to implement a swarm at this time. Secondly, we will be designing a swarm control system for coordinated movement of our quadrotors. Just in case our self-assembled quadrotors are faulty or incomplete, we will also be implementing our swarm system with models in Gazebo. We may choose to create a model that more closely resembles our self-assembled quadrotors if the default model in Gazebo is not comparable. 

## Organization

The first phase of our project will be to design our quadrotors. We will discuss what components will be needed and where to source them from. As it may take a while to obtain all components, we will then begin discussing the design of the swarm system and start development while we are waiting on our components to arrive. Once the components arrive, we will be able to develop the code for them individually, so that we can complete our quadrotors once we have all of the components without any delay. Once they are fully assembled, we will begin the final phase and run real-world tests and perform reliability measurements to ensure they are working as desired. 

- __Phase 1:__ Design quadrotors and source components.
- __Phase 2:__ Create ROS and Gazebo simulation, and begin development on embedded software.
- __Phase 3:__ Assemble quadrotors and perform real-world testing.

If we are unable to construct our quadrotors before the end of the semester, we may elect to source simpler parts and utilize a different type of robot for simple testing.

We will maintain our code base on Github, which will serve as our code repository as well as the project’s website (via Github Pages). Because the project is hosted on Github, we will make the project open-source.

## Measurement

Deadlines will be established for each phase of our project in order to measure our progress. If any section falls behind deadline, more team development time with be shifted to it. While making design discussions, success criteria will be set for each section of the project. These will be evaluated during final testing to determine how successful we were. During the project, our team will maintain a log of our work, primarily using Github, in order to complete a small report at the end of the semester detailing our progress and conclusions about building a multi-quadrotor swarm system. Our own success metric for this project will be gauged by our findings at the end and the simulation demonstrating proof-of-concept of design.

## Cost

We estimate that with all components, the quadrotors will cost in the range of ~$60 per quadrotor. We hope to split the cost between the team members to incur an equal cost for the project.

## Software

We will primarily be using ROS and Gazebo for the development of the swarm control system, which means that the vast majority of our code will be written using C, C++, ROS Configuration Scripts, and possibly a small Python component. Development will primarily be done in a virtual machine shared among the group to ensure compatibility with the target system. To accomplish this, we will be using the free software Vagrant. 
