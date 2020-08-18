# Modular Cartographer

Meta-package based on [cartographer-ros](https://google-cartographer-ros.readthedocs.io/en/latest/index.html)

`cartographer-ros` provides ROS integration for Google's [Cartographer](https://github.com/googlecartographer/cartographer).
Cartographer is a real-time simultaneous localization and mapping (SLAM) system for mobile robots.

Both `cartographer-ros` and `cartographer` have been forked and modified to suit our needs. The fork of `cartographer` is available [here](https://git.web.boeing.com/brta-robotics/cartographer).

## Changes from upstream

Most of the important changes are in `cartographer`:
- Add 2D Submap features
  - Match reflective poles for robust constraint matching
- Add `GlobalICPScanMatcher2D`
  - Fast sampling based global constraint finder
  - Significantly faster than `FastCorrelativeScanMatcher2D` (1s verse 30s)
- Add `ICPScanMatcher2D`
  - Fast dense point matcher
  - Allows for significant deviation from local minima
  - Inclusion of 2d features (in addition to points)
  - Match evaluation based on raytracing and hit (of all points, not just subsampled points)
  - Match evaluation based on feature match
- Optimise `PoseExtrapolator` for wheeled odometry rather than IMU
  - Achieve perfect maps in sim
  - Resolve issues with rotations / poor local mapping
- Remove 3d mapping
- Add heuristics for performant constraint finding
  - Desired number of constraints per submap / trajectory
  - Maximum work queue size
- RangeDataCollator strategy
  - Use a 'one-of-each' strategy rather than time based
- Simplify background task managmenet
  - Remove the complex task queue and thread pool, replace with a single background thread

To create a map from CAD, there is an app `sdf_to_pbstream` that will convert a `.sdf` (or `.world`) file
into a `.pbstream` (protobuf stream), which is how Cartographer likes its maps.

## How to Build

If you just want to use as-is, run
```bash
catkin build modular_cartographer
```

The `CMakeLists.txt` is configured to automagically pull, build and link `cartographer`.
`cartographer` will live in the `build` directory of `cartographer-ros`.

If you are a developer and would like to build `cartographer` manually:

**Build protobuf**
```bash
cd /home/boeing/git
git clone https://github.com/protocolbuffers/protobuf.git
cd protobuf
git checkout v3.4.1
mkdir build
cd build
cmake ../cmake -GNinja -DCMAKE_POSITION_INDEPENDENT_CODE=ON -DCMAKE_BUILD_TYPE=Release -Dprotobuf_BUILD_TESTS=OFF -DCMAKE_INSTALL_PREFIX=install
ninja
ninja install
```

**Build cartographer**
- You need to provide the path to an Abseil tar `ABSEIL_TAR_PATH`
- You need to provide the correct version of protobuf on `CMAKE_PREFIX_PATH`
```bash
cd cartographer
mkdir build
cd build
cmake .. -DABSEIL_TAR_PATH=/home/boeing/ros/robotics_ws/src/modular_cartographer/cartographer_ros/dependencies/abseil-cpp-7b46e1d31a6b08b1c6da2a13e7b151a20446fa07.tar.gz -DCMAKE_PREFIX_PATH=/home/boeing/git/protobuf/build/install -DCMAKE_INSTALL_PREFIX=install -DCMAKE_BUILD_TYPE=RelWithDebInfo -DBUILD_TESTS:BOOL=Off
make -j8
make install
```

**Point cartographer_ros to installation of cartographer**

Modify `cartographer_ros` `CMakeLists.txt` to point to the install path for cartographer
CMakeLists.txt.

Above the line
```bash
if (NOT DEFINED CARTOGRAPHER_INSTALL_DIR)
```
add
```
set(CARTOGRAPHER_INSTALL_DIR /home/boeing/ros/cartographer/build/install)
```

## Tuning Cartographer
Cartographer can be quite sensitive to certain parameters. These are configured in `.lua`
files. A set of default configuration files are available in `cartographer/configuration_files`
although a set of configurations dedicated to the project is most likely required.

Upstream `cartographer_ros` has an excellent [guide](https://google-cartographer-ros.readthedocs.io/en/latest/algo_walkthrough.html) 
on how the algorithm works. The general idea is still relevant, but we have made some enhancements. 

The main area that needs tuning is constraint finding. Constraint finding is the
process of using scan matches to tie submaps together. Constraint finding needs to be
finely tuned because you need to be able to find as many constraints as possible without
getting any false positives. Constraint finding settings are in `map_builder.lua`. 

The there are two types of constraint finding: local and global. A global search is 
performed when the robot has no idea where it is. During a global search, many poses are 
sampled and checked. The number of samples are controlled by `num_global_samples_per_sq_m`
and `num_global_rotations`. Having too few samples will result in a failed search, 
but too many samples will slow down the search.
These samples are filtered based on a number of criteria. 
The scan is compared and a number of metrics such as inlier ratio
are computed. The thresholds for these metrics may need to be tuned to generate
a desired number of proposals.

The proposals are then clustered and then an iterative closest point (ICP) match is
run for each cluster origin. The ICP will produce a score as well as some metrics
for determining whether the match was good. The metrics and its threshold are critical
to filtering out bad constraints. Remember, a bad constraint is worse than no constraint!

The three most useful metrics are:

**Raytrace fraction**

The raytrace score is calculated by tracing a line from the robot to each laser point.
If the line hits an object before reaching the point, it means we are seeing beyond 
an obstacle, so the point fails the raytrace test. The ratio of bad points can be used
to determine if the match was bad. Generally, the raytrace score should be quite high (>0.9)
for a good match. 

Because laser scanners have noise, we allow a margin of error defined by
`raytrace_threshold`. If a point is within this distance from the obstacle, then
we still consider it a pass.

**Hit fraction**

This measures how many points actually hit an object. This ratio will
vary depending on how cluttered the room is and thus may not be very effective
in a complex environment. `hit_threshold` controls the distance between the laser
point and an object for it to register as a hit.

**Features match count**

Features are unique objects in the room such as high-reflective points.
They are very effective in removing ambiguity when the room is very
symmetric. Features are so effective that if the robot sees many features,
we can actually relax the raytrace and hit fraction thresholds. 
Currently, if we see 3 or more features, we reduce the raytrace and hit 
fraction required by 0.025. The parameters `min_hit_fraction` and 
`min_raytrace_fraction` are used when we see 2 or fewer features.

## Testing
### Find constraints
`test_find_constraint` will perform a global constraint search and print pretty
debug images that are extremely helpful with tuning. To run this test, you will need:
- Map in the form of a `.pbstream` file. Generate this by running `sdf_to_pbstream` on the `.world`/`.sdf` map.
- robot `.urdf`
- Rosbag with the laser data

Create a test folder with the `.urdf`, `.pbstream` and the `.bag` files.

In the same directory, create a new directory called
`cartographer_debug`. This name is important! Cartographer will look for this directory
and if it exists, it will save some debug `.png` in it.

Build cartographer_ros and the test app will be in
`devel/.private/cartographer_ros/lib/cartographer_ros/test_find_constraint`

Run it with something like:
```bash
~/catkin_ws/devel/.private/cartographer_ros/lib/cartographer_ros/test_find_constraint --configuration_directory ~/your_project/cartographer_config --pbstream map.pbstream --urdf robot.urdf --rosbag test.bag
```
