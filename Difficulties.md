---
title: Project Difficulties
layout: default
description: Projected difficulties in the Quadrotor SAR project.
---

[Home](https://ece595project.github.io/quadrotor/)

# {{page.title}}

{{page.description}}

## Table of Contents

* This will become a table of contents (this text will be scraped).
{:toc}

## Difficulties

Many of the difficulties we had during this project had to do with hardware. Sourcing components in order to build our own quadrotors was expensive, and with shipping delays, we were unable to build our own quadrotors for a full demonstration.

What we learned was that it is probably a better idea under time constraints to source all components from the same vendor, as the cost savings is offset by a long shipping time from overseas wholesale vendors from sites like eBay.

When we did receive components from these overseas manufactuers, many of the parts were fakes, meaning the specifications and ratings were inaccurate, leading to our power components destroying the flight controller we had purchased. We believe the voltage rating for the power components was too high, leading to our flight controller burning a component. We were able to send both parts back for replacement, but again, the shipping delays between China and the US mean we haven't seen the replacement components yet.

We were able to develop a search algorithm and the code necessary to implement the system in ROS, and we generated a simulation in Gazebo to accompany this report. What remains is testing our system using real quadrotors.

**[back to top](#table-of-contents)**
