# Simple RViz

## Description
The goal of this project is to design a program for managing a mobile robot in ROS while presenting essential system components. The application should include visualizations such as a map obtained from the map server, laser scans, localization particles, and the robot's base depicted as a circular shape. Each component must be displayed in alignment with its respective transformation. The program should also provide functionality for setting an initial position and a destination point to guide the robot's navigation through the planner.

## Installation
Create a folder of your choise (in my case it’s `/home/<usr>/Documents/RobotProgramming`) and download the [srrg catkin workspace](https://gitlab.com/srrg-software) inside of it. I selected only specific package from the original workspace. Run the following code:
```bash
# Create the main workspace directory
mkdir srrg
cd srrg

# Create the source folder where all repositories will be cloned
mkdir src
cd src

# List of repositories to be cloned
repos=(
		srrg2_config_visualizer
		srrg2_core
		srrg2_executor
		srrg2_laser_slam_2d
		srrg2_navigation_2d
		srrg2_orazio
		srrg2_qgl_viewport
		srrg2_slam_interfaces
		srrg2_solver
		srrg_cmake_modules
		srrg_hbst
		srrg_joystick_teleop 
)

# Base URL for cloning the repositories
url_base="https://gitlab.com/srrg-software"

# Iterate through the list and clone each repository
for repository in "${repos[@]}"; do
    echo "Downloading $repository..."
    git clone "$url_base/$repository.git"
done

# Clone the test_20.04 branch of the srrg2_webctl repository
git clone -b test_20.04 "https://gitlab.com/srrg-software/srrg2_webctl.git"

echo "All repositories have been successfully downloaded."

```
The next step involves setting up and launching ROS. For this project, I chose to use a Docker image of ROS Noetic. This decision was influenced by the fact that my system runs Ubuntu 22.04, which primarily supports ROS 2. Using Docker allowed me to emulate a ROS Noetic environment without interfering with my existing setup.

To run the Docker container with GUI support for visualizing applications, share the catkin workspace directory located on the host machine at `/home/<usr>/Documents/RobotProgramming/srrg`, and enable tmux (a Terminal Multiplexer), the following command can be used:

```bash
# Download docker image
docker pull ros:noetic-robot
# Give Docker access to display
xhost +local:docker
# Run docker image with tmux (change <usr>)
docker run -it --rm \
    --name ros_gui_container \
    -e DISPLAY=$DISPLAY \
    -v /tmp/.X11-unix:/tmp/.X11-unix \
	  -v /home/leonardo/Documents/RobotProgramming/srrg:/root/catkin_ws \
    ros:noetic-robot \
    /bin/bash -c "apt update && apt install -y tmux && tmux new"
```
At this point, we can move on to installing the required dependencies. Most of these are listed in the `setup.sh` file found in the [labiagi git repository](https://gitlab.com/grisetti/labiagi_2020_21.git). This step is necessary because certain libraries, such as OpenCV, are not pre-installed in the `ros:noetic-robot` Docker image.
```bash
# Specify the ROS version being used
ROS_DISTRO="noetic"

# Declare the list of necessary libraries and tools
DEPENDENCIES=" \
  libeigen3-dev \
  libsuitesparse-dev \
  ninja-build \
  libncurses5-dev \
  libwebsockets-dev \
  libreadline-dev \
  libbullet-dev \
  libopencv-dev \
  libpcl-dev \
  libyaml-cpp-dev \
  qtdeclarative5-dev \
  qt5-qmake \
  libqglviewer-dev-qt5 \
  libudev-dev \
  freeglut3-dev \
  libgtest-dev \
  libglfw3-dev \
  python3-catkin-tools \
  python3-rosdep \
  python3-rosinstall \
	python3-rosinstall-generator \
	python3-wstool \
  build-essential \
  ros-${ROS_DISTRO}-grid-map-msgs \
  ros-${ROS_DISTRO}-image-transport \
  ros-${ROS_DISTRO}-cv-bridge \
  ros-${ROS_DISTRO}-turtlesim \
  ros-${ROS_DISTRO}-stage-ros \
  ros-${ROS_DISTRO}-rviz \
  ros-${ROS_DISTRO}-tf2 \
  ros-${ROS_DISTRO}-tf2-ros
"

# Load ROS environment variables
ROS_SETUP="/opt/ros/${ROS_DISTRO}/setup.bash"
source ${ROS_SETUP}

# Set up and update rosdep for managing dependencies
sudo rosdep init
rosdep update

# Update the package lists and install all required packages
sudo apt update
sudo apt-get install -y ${DEPENDENCIES}
```
Now we need to build the ROS workspace:
```bash
# source setup file (always)
source /opt/ros/noetic/setup.bash
# Initialize and build workspace
cd ~/catkin_ws/src
catkin_init_workspace
cd ..
catkin build -j$(expr $(nproc) / 2) -p$(expr $(nproc) / 2)
# source workspace (always)
source ~/catkin_ws/devel/setup.bash

```
To preserve the Docker image with all the installed dependencies, you can create a new image by committing the container. This ensures that all the installed packages and configurations are saved, so you won't need to reinstall them each time you launch the container. Open a new terminal on your local machine and run the following command to commit the container and create a new image named `ros:noetic-rp` (every name chosen will be fine):

```bash
docker commit ros_gui_container ros:noetic-rp
```
At this point, we can shut down the current Docker container by typing `exit` in all open terminals.

Once the Docker image is committed, the next step is to clone the repository into the working directory (`/home/<usr>/Documents/RobotProgramming`). To accomplish this, simply run the following command:

```bash
# Move into the working directory
cd /home/leonardo/Documents/RobotProgramming
# Clone the SimpleRViz GitHub repository
git clone https://github.com/LeonardoMontella/RobotProgramming.git
```
This will create the `/home/<usr>/Documents/RobotProgramming/RobotProgramming` directory on your local machine.
Rename the folder writing this command:
```bash
mv RobotProgramming SimpleRViz
```
So now you will have `/home/<usr>/Documents/RobotProgramming/SimpleRViz`.

If there's an issue with a package and the localize and planner nodes cannot start, you can automatically fix the `dl.conf` file by running the following script. This will update the file and save it to the root directory:

```bash
# Specify the full paths for the config file and shared object directory
CONFIG_FILE="$HOME/dl.conf"
SO_PATH="$HOME/catkin_ws/devel/lib"

# Find all .so files in the specified directory
SO_FILES=($(find "$SO_PATH" -maxdepth 1 -name "*.so"))

# Generate and write the configuration file
{ \
  echo "\"DynamicLoaderConfig\" {"; \
  echo "  \"so_names\" : ["; \
  for FILE in "${SO_FILES[@]}"; do \
    FILENAME=$(basename "$FILE"); \
    echo "    \"$FILENAME\","; \
  done | sed '$ s/,$//'; \
  echo "  ],"; \
  echo "  \"so_paths\" : [ \"$SO_PATH\" ]"; \
  echo "}"; \
} > "$CONFIG_FILE"

echo "The configuration file has been updated and saved at $CONFIG_FILE"
``` 

This will automatically update the configuration file to ensure that the necessary libraries are properly loaded.

## Start and use th Docker Image 

Now, you can configure Docker to mount both workspaces: `srrg` as the primary workspace at `/root/catkin_ws` and `SimpleRViz` at `/root/catkin_ws_rp`. Additionally, we map the container's port `9001` to the host's port `9001` for `webctl`. This setup enables you to work with and run ROS nodes from both workspaces independently. 

I also added `--env="LIBGL_ALWAYS_SOFTWARE=1"` to force the software rendering of Stage, as it was not starting otherwise.

Here’s the command to launch the Docker container:

```bash
# Allow Docker to access the display for GUI applications
xhost +local:docker

# Run the Docker container, mounting both workspaces and setting the necessary environment variables
docker run -it --rm \
    --name ros_gui_container \
    -e DISPLAY=$DISPLAY \
    --env="LIBGL_ALWAYS_SOFTWARE=1" \  # Force software rendering for Stage
    -v /tmp/.X11-unix:/tmp/.X11-unix \
    -v /home/leonardo/Documents/RobotProgramming/srrg:/root/catkin_ws \
    -v /home/leonardo/Documents/RobotProgramming/SimpleRViz:/root/catkin_ws_rp \
    -p 9001:9001 \
    ros:noetic-rp \
    /bin/bash -c "tmux new"
```
This command starts the saved image preserving all installed packages and the linked shared folder.

To move between the two workspaces, you can now use:

```bash
# Move to workspace
cd ~/catkin_ws/
# Move to the SimpleRViz workspace
cd ~/catkin_ws_rp
```

Always remember to source all workspaces:

```bash
# Source workspace
source /opt/ros/noetic/setup.bash 
source ~/catkin_ws/devel/setup.bash
source ~/catkin_ws_rp/devel/setup.bash
```
Remember that you can always save the changes with:

```bash
docker commit ros_gui_container ros:noetic-rp
```

## How to Use

To launch `webctl`:

```bash
# Set up the ROS environment
source /opt/ros/noetic/setup.bash 
source /root/catkin_ws/devel/setup.bash

# Build the webctl package
cd ~/catkin_ws/src/srrg2_webctl
make

# Start the webctl service
cd ~/catkin_ws/src/srrg2_navigation_2d/config/ 
~/catkin_ws/src/srrg2_webctl/proc_webctl run_navigation.webctl
```

Then, navigate to [localhost](http://localhost:9001) in your browser. On the `webctl` page, start `roscore`, `stage`, and `mapserver`.

Next, in a new terminal, launch the `simple_rviz` node by running:

```bash
# Set up the ROS environment
source /opt/ros/noetic/setup.bash 
source /root/catkin_ws/devel/setup.bash
source ~/catkin_ws_rp/devel/setup.bash

# Run the simple_rviz node
roscd simple_rviz/
rosrun simple_rviz simple_rviz
```

Finally, in the `webctl` interface, start the `localize`, `planner`, and `follower (static)` processes.

On the map generated by the node you will need to follow the written instructions: Press 'I' and click on a desired point on the map to select the initial pose of the robot. Then press 'G' and click on the map to select the goal position.
The robot will reach the goal point without colliding with walls.

## Project Overview

**Note:** These steps are not necessary if you’ve already cloned the repository into `~/catkin_ws_rp`.

To set up this project, I started by creating a new package named `simple_rviz`, which depends on `roscpp` and `std_msgs`, using the following command:

```bash
# Source the ROS workspace
source /opt/ros/noetic/setup.bash 
source /root/catkin_ws/devel/setup.bash

# Create the catkin package
cd ~/catkin_ws_rp/src
catkin_create_pkg simple_rviz std_msgs roscpp
```

Next, navigate to the `simple_rviz/src` directory and create a `simple_rviz.cpp` file:

```bash
# Navigate to the src directory and create the C++ file
cd simple_rviz/src
touch simple_rviz.cpp
```

At this point, you can edit the file locally by accessing it from your shared folder.

Don't forget to include the necessary lines in the `CMakeLists.txt` file:
```bash
## Find OpenCV
find_package(OpenCV REQUIRED)

## Include the headers
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}  # Include OpenCV headers
)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  nav_msgs
  tf2
  tf2_ros
)

## Declare a catkin package
catkin_package(
  CATKIN_DEPENDS 
  roscpp 
  std_msgs 
)

## Add an executable
add_executable(simple_rviz src/simple_rviz.cpp)

## Link the executable to the required libraries
target_link_libraries(simple_rviz 
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}  # Link OpenCV libraries
)
```
Now, return to the root of your workspace, build the workspace, and run the node using the following commands:

```bash
# Source the ROS setup file (this step is always required)
source /opt/ros/noetic/setup.bash

# Initialize and build the workspace
cd ~/catkin_ws_rp/src
catkin_init_workspace
cd ..
catkin build -j$(expr $(nproc) / 2) -p$(expr $(nproc) / 2)

# Source the workspace again (this step is always required)
source ~/catkin_ws_rp/devel/setup.bash

# Run the node
rosrun simple_rviz simple_rviz
```

These commands will initialize, build, and run the node.
