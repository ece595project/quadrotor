---
title: Project Report
layout: default
description: Quadrotor SAR project report.
---

[Home](https://ece595project.github.io/quadrotor/)

# {{page.title}}

{{page.description}}

## Table of Contents

* This will become a table of contents (this text will be scraped).
{:toc}

## Report

In this report, you will find information on the design decisions we made on our project and find information on some of the main points not detailed in the other sections.

In the two months we had to complete this project after we finalized the initial project proposal, we spent a large amount of our time figuring out how to implement our quadrotor on actual hardware, which left less time for including more features, such as image recognition for the people who are in need of rescue.

### Thermal Camera and Image Recognition

In order to detect people in an environment, it would have been useful to utilize a FLIR camaera on the quadrotor in order to do the image recognition of "hot spots". This, coupled with an image recognition library such as OpenCV would have enabled us to detect stranded hikers easier. However, with the time constraints of the project, we decided that it would be easier to focus on the search algorithm rather than the actual implementation of image recognition that this would require.

Here is a video demonstrating IR camera footage from a quadrotor:

{% include youtubePlayer.html id="AtbhifOf6n4" %}

### GPS Waypoints

Because our solution relies heavily on GPS locating directly on the individual quadrotors, we knew that our solution would be primarily used outdoors, where GPS data is easily obtained. GPS dictates much of the path planning for the quadrotor, and determines the waypoints the quadrotor needs to head toward as it moves.

We chose GPS waypoints because we do not know the terrain beforehand, and the search and rescue workers won't know the terrain beforehand, either. However, given a starting location, the search and rescue workers would be able to send the quadrotors off from their starting position, and they would be able to return to the same location once they had found the stranded person or when they were low on power. This enables one person to track the quadrotors from a single position as other members of the team head off to search on the ground.

Using GPS waypoint tracking as our path-following algorithm seemed to be good enough for our needs. You can see images of the quadrotors performing GPS waypoint tracking in simulation below.

![Path Following 1]({{ site.url }}/quadrotor/assets/path_following_1.png)

![Path Following 2]({{ site.url }}/quadrotor/assets/path_following_2.png)

Search and rescue workers often send people off in random directions where they anticipate an injured or stranded person to be, hoping to get as close as possible, while the rest of the team waits for reports from the 'scouts'. Using quadrotors as scouts is faster and more effective than the current approach. Also, due to the fact that the quadrotors are aerial vehicles, they can search an area more effecively and find people easier than a ground-based approach.

### Building Quadrotors

One of the major problems we had with this project was cost. While we would have liked to have been able to spend more money to source a pre-made quadrotor, we were limited by the fact that quadrotors are prohibitively expensive when features such as cameras and GPS are added. Thermal cameras and GPS modules easily push the cost of a single quadrotor into the range of a "premium" product, costing well upwards of $1000. Because we are students on a budget, purchasing a quadrotor at this price point was not possible.

Initially, we had thought to build a flight controller "hat" for the Raspberry Pi Zero, with a GPS module on-board, but the constraints on our project meant that we had to focus on other features of the project rather than focusing on what we considered to be "stretch goals."

**[back to top](#table-of-contents)**
