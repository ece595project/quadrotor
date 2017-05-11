We considered several different search algorithms for our project, including cartesian search grids, decentralized waypoint searches, and spiral search paths, considering the fact that our quadrotors could search a large area unhindered by terrain.

What we decided to implement in our system, despite the fact that it may be less efficient, is a sinusoidal path in polar coordinates, which allowed us to increase the number of quadrotors arbitrarily and to designate a general trajectory while maximizing search coverage of an area.

![Polar Sine]({{ site.url }}/quadrotor/assets/polar_plot.png)

We decided that using polar coordinates was desirable, mainly due to the fact that the quadrotors would not always be in range to communicate with each other or with the rescuer. We first investigated whether or not a search path such as the one below was adequate for our needs, but felt there was too much area that would be left unsearched.

![Polar Sine]({{ site.url }}/quadrotor/assets/polar_sine.png)

```Matlab
theta = 0:0.01:2*pi;
rho = sin(2*theta).*cos(2*theta);
```

By setting the radial coordinate equal to the angle, we could create a spiral pattern for the search and rescue quadrotors, varying the coefficients for each successive quadrotor to ensure they don't overlap (something like 0.4, 0.7, 1.7, 2.7, etc.), but we would sacrifice a lot of time flying over areas that are close or very near to areas that other quadrotors have already searched, and a person in need of rescue would see potentially several quadrotors fly nearby, but not necessarily in the path necessary to detect them.

![Polar Spiral]({{ site.url }}/quadrotor/assets/polar_spiral.png)

```Matlab
theta = 0:0.01:8*pi;
rho = theta;
```
