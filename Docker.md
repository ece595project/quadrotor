---
title: Docker
layout: default
description: Description of Docker integration into ROS.
---

# {{page.title}}

{{page.description}}

[Home](https://ece595project.github.io/quadrotor/)

## Table of Contents

* This will become a table of contents (this text will be scraped).
{:toc}

## Using Docker With ROS

One major hurdle we faced in our project was a way to implement a consistent development environment with ROS. Unfortunately, ROS versioning can be problematic, and their development workspaces do not promote rapid development. Furthermore, they also suffer from the problem of one coder saying, "well, it worked on my system."

To solve this problem, we decided to take advantage of a little-known feature of ROS integrations with a novel method for development using [Docker](https://www.docker.com).

## What is Docker?

Docker is a virtualization system that allows people to package applications in containers. Containerization is a technology that has been available on Linux for a long time, but the mainstream usage of Docker for development has only been discovered in the past few years.

Docker allows you indirectly to package a consistent ROS environment into a single file, allowing you to ensure that every member of the development team is using the same development environment.

## ROS + Docker

So how does using ROS in Docker work? Well, Docker provides "images", which can be thought of as a pseudo-virtual machine, that ensures a consistent build of an image each time you type "docker run". What this means is that Docker is able to run an instance of ROS with a single command, that ensures that your application will work on any computer, every time.

One of the major benefits of using ROS inside a Docker container is that Docker images are pre-built binaries, meaning you can upload and download an image quickly, rather than having to install ROS on each individual machine, ROS is installed once inside the image, and can be compressed and pushed around to any machine running Docker, and run again, in a matter of seconds. What this also means is that running an instance of docker can easily be scaled multiple times on the same computer or network, giving you any number of instances of the same application, almost instantly.

In 2015, ROS provided weak support for Docker by providing Docker "images" for ROS, available of [Docker Hub](https://hub.docker.com). Since the initial release of the Docker images in 2015, they have provided sporadic updates to the Docker images, and do not provide a consistent release cycle for new versions of ROS, but have included Gazebo images along with their ROS images. See more on the [ROS website](http://wiki.ros.org/docker/Tutorials/Docker).

## What are the drawbacks?

Unfortunately, as with much of ROS, there is little to no documentation on how to utilize the Docker images properly, which means that you are, as per usual, on your own when it comes to figuring out how to use ROS inside the Docker images properly.

## Our Approach

Luckily, we are able to utilize the same code base for our Docker image for every quadrotor. Since each quadrotor performs nearly the same actions, we can reuse the code from one quadrotor and apply it to as many instances (or individual quadrotors) as we have available. Because swarm tactics dictates that each quadrotor performs its tasks nearly independently of any centralized hub, we are able to take advantage of ad-hoc networking and Docker to create the behavior for each quadrotor.
