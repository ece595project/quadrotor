---
title: Project Testing
layout: default
description: Testing of the Quadrotor SAR project code.
---

[Home](https://ece595project.github.io/quadrotor/)

# {{page.title}}

{{page.description}}

## Table of Contents

* This will become a table of contents (this text will be scraped).
{:toc}

## Quadrotor Testing

We were able to perform our initial testing of the integration between ROS and an actual quadrotor device using a borrowed quadrotor from a coworker with access to a quadrotor that had GPS capabilities and an FPV camera. This coworker was able to help us to program the flight controller to publish ROS topics via minimal code on the quadrotor system. However, due to the memory constraints of the system, we were only able to receive those topics while the quadrotor was connected to a computer via a serial cable.

One thing we had hoped to achieve with our system was to use a Raspberry Pi Zero as the 'brain' of the quadrotor in order to allow us to install ROS on the quadrotor itself, meaning the quadrotor would be able to publish topics over WiFi or via bluetooth connectivity.

![Published Data]({{ site.url }}/quadrotor/assets/published_data.png)

What this told us initially was that we would need to either buy or design a flight controller that was compatible with the Raspberry Pi Zero, and some commercial solutions do exist, such as the [PXF Mini](http://erlerobotics.com/blog/product/pxfmini/) by Erle Robotics. However, we were again prohibited by cost, as the PXF Mini is approximately $75, which would be added on to the total cost of the system, pushing our costs to approximately $375 per quadrotor, eliminating the benefits of building a quadrotor ourselves. This would have been better than the ~$800 quadrotor available directly from Erle Robotics, but would have still been more expensive than some of the other quadrotors we sourced.

See the video below to see the PXF Mini in action.

{% include youtubePlayer.html id="LWwkB6hGD4M" %}

## Device Testing

After briefly working with the quadrotor to experiment with publishing topics using data gained from the sensors, we decided that we would like to look at making sure we could publish data from GPS, specifically, in close to real-time so that we could track the location of our quadrotors when a person in the search area was detected.

We were able to connect a GPS module to an Arduino, and via the connection to the serial cable, we were able to read the data and replicate the process we had performed before using the borrowed quadrotor. We began testing whether or not we could receive the GPS data using a direct connection to a Raspberry Pi Zero, but were unable to finish this process due to time constraints.

## Gazebo Testing

We created some simulations of our quadrotor's movement characteristics in order to get an idea of how our quadrotors would be able to synchronize their movement in a simulated environment.

For more information on the search pattern design, see the documentation on the [search algorithm](https://ece595project.github.io/quadrotor/Search-Algorithm).


**[back to top](#table-of-contents)**
