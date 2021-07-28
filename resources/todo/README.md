
### Issue 1: Scan-matcher

There seems to be an issue that the scan-matcher does not return probable transformation between two point clouds.

Scan-matching tries to incrementally align two scans or a map to a scan, without revising the past/map.

* PRO TIP: Outlier detection. Remove outliers from the pointcloud

* During filtering, it can happen that the scan-matching process fails because of poor observations or a too small
  overlapping area between the current scan and the previously computed map
* Add more tests and compare with the Python ICP algorithm
* Try running scan matcher only every 0.5m or 25 degree rotation
* Investigate limiting the search area of the scan-matcher
* Try tuning the parameters of the ICP scan-matching function

### Issue 2: the observation probability distribution function is really slow

* Do not iterate over whole map at for all sampled poses. Store occupied cells for each particle

### Issue 3: motion model distribution might be wrong


### Issue 4: map gradually diverges

* Make sure the resampling method is correct
* Have a look at the python RBPF SLAM implementation and compare the functions to look for errors

### Issue 5: Simulation of laser

* If you constantly turn the robot, the laser scan produces an offset. This could be a potential
source of error