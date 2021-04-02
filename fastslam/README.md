
## The occupancy grid mapping algorithm

**Occupancy grid mapping** addresses the problem of generating consistent maps from noisy and uncertain measurement data,
under the assumption that the robot pose is known. The basic idea of occupancy grids is to represent the map as a field
of random variables, arranged in an evenly spaced grid. Each random variable is binary and corresponds to the occupancy
of the location it covers. Many SLAM techniques do not generate maps fit for path planning and navigation. Occupancy grid
maps are often used after solving the SLAM problem by some other means, and taking the resulting path estimates for granted.

The gold standard of any occupancy grid mapping algorithm is to calculate the posterior over maps given the data

<img src="https://latex.codecogs.com/gif.latex?p(m|z_{1:t},&space;x_{1:t})" />




In the above equation, *m* is the map, <img src="https://latex.codecogs.com/gif.latex?%5Cinline%20x_%7B1%3At%7D" />