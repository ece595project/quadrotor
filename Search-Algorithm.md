---
title: Search Algorithm
layout: default
description: Search algorithm design for the Quadrotor SAR project.
---

# {{page.title}}

{{page.description}}

[Home](https://ece595project.github.io/quadrotor/)

## Table of Contents

* This will become a table of contents (this text will be scraped).
{:toc}

## Search Algorithm Design

We considered several different search algorithms for our project, including cartesian search grids, decentralized waypoint searches, and spiral search paths, considering the fact that our quadrotors could search a large area unhindered by terrain.

What we decided to implement in our system, despite the fact that it may be less efficient, is a 'sinusoidal' path in polar coordinates, which allowed us to increase the number of quadrotors arbitrarily and to designate a general trajectory while maximizing search coverage of an area.

![Polar Sine]({{ site.url }}/quadrotor/assets/polar_plot.png)

## Polar Sine Pattern

We decided that using polar coordinates was desirable, mainly due to the fact that the quadrotors would not always be in range to communicate with each other or with the rescuer. We first investigated whether or not a search path such as the one below was adequate for our needs, but felt there was too much area that would be left unsearched.

![Polar Sine]({{ site.url }}/quadrotor/assets/polar_sine.png)

```Matlab
theta = 0:0.01:2*pi;
rho = sin(2*theta).*cos(2*theta);
```

## Spiral Pattern

By setting the radial coordinate equal to the angle, we could create a spiral pattern for the search and rescue quadrotors, varying the coefficients for each successive quadrotor to ensure they don't overlap (something like 0.4, 0.7, 1.7, 2.7, etc.), but we would sacrifice a lot of time flying over areas that are close or very near to areas that other quadrotors have already searched, and a person in need of rescue would see potentially several quadrotors fly nearby, but not necessarily in the path necessary to detect them.

![Polar Spiral]({{ site.url }}/quadrotor/assets/polar_spiral.png)

```Matlab
theta = 0:0.01:8*pi;
rho = theta;
```

## Proposed Solution

What we decided in the end, which was in our opinion the fastest method with the most ground coverage, was an expanding wave in polar coordinates that emanated from the origin, bounded by two angles determined at the start by the number of quadrotors in the swarm. This means that as the number of quadrotors increases, we achieve better and faster coverage of an area, with little overlap, and are able to arbitrarily increase the number of search quadrotors.

![Polar Search 1]({{ site.url }}/quadrotor/assets/polar_search_1.png)

![Polar Search 2]({{ site.url }}/quadrotor/assets/polar_search_2.png)

```Matlab
numQuadrotors = 7;
a = 3*(1/numQuadrotors);

b = 180;
b_rad = deg2rad(b);

figure
polarplot(rho, theta)
for i = 1:numQuadrotors
    c = i*(360/numQuadrotors);
    c_rad = deg2rad(c);

    rho = 0:0.1:10;
    theta = a*sin(b_rad*rho)-c_rad;

    polarplot(theta,rho)
    hold on
end

hold off
```

We can see from the code that we have three parameters which we can alter as part of the equation governing the search parameters. We can vary `a` to determine the 'amplitude' of the sine wave, increasing the search area and angle that a single quadrotor will be able to search, we can vary the angular frequency, `b`, in order to increase coverage of the wedge that a single quadrotor will search, and we can vary `c` to change the angle the sine wave will follow.

If we hold `c` to be within a certain number of values, we can direct the search along a particular path, such as within a cone, rather than a circular area around the origin.

When we increase the number of quadrotors, we need to change the `a` constant to a value that is able to give us a wide enough search area that has no overlap.

## Search Algorithm In Action

See the video below, which is also on our home page, to see what the search algorithm looks like in a Gazebo simulation.

{% include youtubePlayer.html id="PK7pUsQQ6ss" %}
