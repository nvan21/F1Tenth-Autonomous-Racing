# F1Tenth Autonomous Racing Algorithm

This repository uses the [UPenn F1Tenth simulator](https://github.com/f1tenth/f1tenth_simulator?tab=readme-ov-file) and ROS to simulate autonomous driving around the Nurburgring race track. The goal of the project was to drive two laps: one as fast as possible, and one as smooth as possible. The final algorithm I used was a modified follow the gap and the brainstorming process is documented below.

![Video of the smooth lap](/Smooth%20Lap.gif)

# Brainstorming

There were multiple approaches that I thought of when I first started brainstorming solutions to the problem, but the main two that I considered were path
planning with RRT\* and a dynamic follow the gap algorithm using LiDAR data.

### SLAM and RRT\*

In an interest to try implementing something I’d never done before, I started
with trying to combine hector mapping and RRT\* to create a dynamic path
planning algorithm. However, there were multiple issues that I had to solve if
I wanted to implement this solution. The first one is that the starting vertex
and goal vertex are the same since it’s a circular track. This means that the
map needs to be split up into multiple smaller sections. The most optimal way
to dynamically partition the track wasn’t clear to me, so I decided that picking
the farthest point on the LiDAR scan would work.
Although unlikely, there is an edge case where the tree would explore the
wrong way and still end up at the goal node. For example, consider a square
track where the start node is in the top left corner and the goal node is in the
top right corner. The tree is most likely to branch out along the top side and
find the goal, but there is a chance that it goes down the left side, across the
bottom, and then up the right side which would not be optimal. To remedy
this, I created a node field of view. This means that nodes can only generate
within a certain angle of the car which eliminates that edge case. It also has the
added benefit of being computationally quicker since it’s more of an informed
random tree rather than a truly random one.
The next issue is that the hector map has a high resolution. While it’s helpful
for accuracy, it increases computational complexity and is still a discretized
representation of a continuous space. My algorithm tweak for solving the above
problem means that it needs to generate the node position in continuous polar
coordinates and convert those coordinates to a discrete point. Then it has to
calculate whether the line connecting the new node and the nearest node crosses
the track boundaries, so it has to use Bresenham’s line algorithm to convert a
continuous line to a discretized vector of points. Once it has these points, it
has to compare it with the array of all other obstacle points to see if there are
1
any matches. All of this becomes very computationally expensive which is not
desirable on a fast moving robot. One optimization that I came up with would
be to prune all obstacle points not within the node field of view.
The last major issue that I thought of was that the starting node was also
dynamic since the robot is moving while the algorithm is running. It wasn’t
entirely clear how I would account for this without major optimizations to my
original algorithm. With deadlines for other classes approaching, this issue is
what tipped the scale and led me to implementing a different solution.

### Dynamic Follow the Gap

My other idea was to cleverly parse through LiDAR data using a few heuristics. My original fear with implementing this approach was that it would not
produce a smooth trajectory, but the algorithm is computationally cheap, so it
is able to take advantage of the high update rate of the LiDAR and produce a
surprisingly smooth trajectory.
The basis of the algorithm is that a good way to get close to the global
optimum lap time is to drive straight at the top speed for as long as possible.
This means that the car should always be pointed at the angle which has the
farthest away LiDAR point. If the car was a point, this algorithm would be
finished. However, the algorithm needs to account for the physical dimensions
of the car. The idea of driving towards the farthest point will still work, but
the LiDAR data needs to be filtered first.
The filtering works by detecting discrepancies between adjacent scans. If
the difference between the two scan distances is more than some user-defined
threshold, then the algorithm should go back and set a certain amount of scans
to be the lower distance. This means that the algorithm will not try to go to
a point that will make the side of the car crash into the wall. It does this by
treating the width of the car as one side of a right triangle with the LiDAR scan
as the hypotenuse. It then calculates the angle that this makes from the car
and converts that to a number of scans + some arbitrary conservative threshold.
This new filtered data is then parsed for the maximum, and the steering angle
is set to either point the car towards the target point or the maximum steering
angle. This algorithm does not account for the side of the car hitting a wall
while it turns, so there is an extra check to make sure that there are no points
on the side of the car within a user-defined safety distance before turning.
Intuitively, the car should travel at a slower speed when the point directly
in front of it is close and at a faster speed when that point is farther away.
Although my implementation of the algorithm gives the user a choice between
a smooth mode and a fast mode, both use piece-wise functions to decide the
speed. My specific implementation is just a step function of the distance in
front of the car, but it could be made smoother by changing the steps to some
linear function of that distance.

# References

- [RRT and RRT\* Explanation](https://theclassytim.medium.com/robotic-path-planning-rrt-and-rrt-212319121378#:~:text=RRT*%20is%20an%20optimized%20version,to%20develop%20a%20shortest%20path)
- [Bresenham Line Algorithm Implementation in Python](https://babavoss.pythonanywhere.com/python/bresenham-line-drawing-algorithm-implemented-in-py)
- [UNC 2019 F1 Tenth Member Explaining their Algorithm](https://www.nathanotterness.com/2019/04/the-disparity-extender-algorithm-and.html)
