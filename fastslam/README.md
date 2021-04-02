
## The SLAM Problem

<img src="https://d3i71xaburhd42.cloudfront.net/f29bff9cf75ed6f632cead2a57552f03e7188df6/12-Figure37.8-1.png" alt="drawing" style=" width: 300px:" />

From a probabilistic perspective, there are two main forms of the SLAM problem, which are both of equal practical importance.
One is known as the *online SLAM Problem* and the *full SLAM Problem*

#### Online SLAM Problem

The online SLAM problem involves estimating the posterior over the momentary pose along with the map

<img src="https://latex.codecogs.com/gif.latex?p(x_t&space;,&space;m&space;|&space;x_{t})" /> 

Here <img src="https://latex.codecogs.com/gif.latex?\inline&space;x_{t}" /> is the pose at time *t*, *m* is the map,
and <img src="https://latex.codecogs.com/gif.latex?\inline&space;z_{1:t}" /> and 
<src img="https://latex.codecogs.com/gif.latex?\inline&space;u_{1:t}" /> are the measurements and controls, respectively.
The problem is called the online SLAM problem since it only involves the estimation of variables that persist at time *t*.
Many SLAM problems are incremental, meaning they discard past measurements and controls once they have been processed.

## FastSLAM algorithm for Occupancy Grid Maps

The FastSLAM algorithm uses particle filters (*Rao-Blackwellized particle filters*) for estimating the robot path. This means
that for each of these particles (believes) the individual map errors are *conditionally independent*. Hence the mapping problem
can be factored into many separate problems, one for each feature in the map. FastSLAM estimates these map feature locations by EKFs
, but using a separate low-dimensional EKF for each individual feature. This is fundamentally different from most SLAM
algorithms which tend to all use a single Gaussian distribution to estimate the location of all features jointly.

#### Advantages of FastSLAM

The key advantage of FastSLAM, however, stems from the fact that data association (determining the correct mapping
of observations to landmark/particle) decisions can be made on a per-particle basis.
As a result, the filter maintains posteriors over multiple data associations, not just the most likely one. This is in stark
contrast to most SLAM algorithms, which track only a single data association at any point in time. In fact, by sampling over
data associations, FastSLAM approcimates the full posterior, not just hte maximum likelihood data association. The ability
to pursue multiple data associations simultanesouly makes FastSLAM significantly more robust to data assosciation problems
than algorithms based on incremental maximum likelihood data association. Another advantage of FastSLAM over other SLAM
algorithms arises from the fact that particle filters can cope with non-linear robot motion models, whereas previous
techniques approximate such models via linear functions. This is important when the kinematics are highly non-linear, or when
the pose uncertainty is relatively high.

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