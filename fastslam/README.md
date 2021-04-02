
## The Occupancy Grid Mapping Algorithm

**Occupancy grid mapping** addresses the problem of generating consistent maps from noisy and uncertain measurement data,
under the assumption that the robot pose is known. The basic idea of occupancy grids is to represent the map as a field
of random variables, arranged in an evenly spaced grid. Each random variable is binary and corresponds to the occupancy
of the location it covers. Many SLAM techniques do not generate maps fit for path planning and navigation. Occupancy grid
maps are often used after solving the SLAM problem by some other means, and taking the resulting path estimates for granted.

The gold standard of any occupancy grid mapping algorithm is to calculate the posterior over maps given the data

<img src="https://latex.codecogs.com/gif.latex?p(m|z_{1:t},&space;x_{1:t})" />


In the above equation, *m* is the map, <img src="https://latex.codecogs.com/gif.latex?\inline&space;z_{1:t}" /> the set of
all measurements up from 1 to time *t*, and <img src="https://latex.codecogs.com/gif.latex?\inline&space;x_{1:t}" /> is the path of
the robot defined through the sequence of all poses, 1 to time *t*. The controls <img src="https://latex.codecogs.com/gif.latex?\inline&space;u_{1:t}" />
play no role in the ocucpancy grid maps, since the path is already known. 

The types of maps considered by occupancy grid maps are fine-grained grids defined over the continous space of locations.
Let <img src="https://latex.codecogs.com/gif.latex?\inline&space;m_{i}" /> denote the grid cell with index *i*. An occupancy
grid map partitions the space into finitely many grid cells:

<img src="https://latex.codecogs.com/gif.latex?m&space;=&space;\{&space;\boldsymbol{m}_i&space;\}" />

Each <img src="https://latex.codecogs.com/gif.latex?\inline&space;\boldsymbol{m}_i" /> has attached to it a binary
occupancy value, which specifies whether a cell is occupied or free. We will write "1" for occupied and "0" for free.
The notation <img src="https://latex.codecogs.com/gif.latex?\inline&space;p&space;(\boldsymbol{m}_i&space;=&space;1)" />
or <img src="https://latex.codecogs.com/gif.latex?\inline&space;p&space;(\boldsymbol{m}_i)" /> refers to the probability
that a grid cell is occupied.

The standard occupancy grid approach breaks down the problem of estimating the map into a collection of separate problems,
namely that of estimating

<img src="https://latex.codecogs.com/gif.latex?p&space;(\boldsymbol{m}_i&space;|&space;z_{1:t},&space;x_{1:t})" />

for all grid cell <img src="https://latex.codecogs.com/gif.latex?\inline&space;m_{i}" />. Each of these estimation problems is now
a binary problem with static state. This decomposition is convenient but not without problems. In particular, it does not
enable us to represent dependencies among neighboring cells; instead, the posterior over maps is approximated as the product of its marginals:

<img src="https://latex.codecogs.com/gif.latex?p&space;(m_i&space;|&space;z_{1:t},&space;x_{1:t})&space;=&space;\prod_{i}&space;p&space;(\boldsymbol{m}_i&space;|&space;z_{1:t},&space;x_{1:t})" />

