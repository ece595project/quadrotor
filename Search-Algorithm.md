We considered several different search algorithms for our project, including cartesian search grids, decentralized waypoint searches, and spiral search paths, considering the fact that our quadrotors could search a large area unhindered by terrain.

What we decided to implement in our system, despite the fact that it may be less efficient, is a sinusoidal path in polar coordinates, which allowed us to increase the number of quadrotors arbitrarily and to designate a general trajectory while maximizing search coverage of an area.

![Polar Sine]({{ site.url }}/assets/polar_plot.png)

We decided that using polar coordinates was desirable, mainly due to the fact that the quadrotors would not always be in range to communicate with each other or with the rescuer. We first investigated whether or not a search path such as the one below was adequate

![Polar Sine]({{ site.url }}/assets/polar_sine.png)

```Matlab
theta = 0:0.01:2*pi;
rho = sin(2*theta).*cos(2*theta);
```
