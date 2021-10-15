# Item preparation suite

A collection of packages with the aim to easily add transformations to things.

## What to do first (skippable):
1. Install ros:
```
https://docs.ros.org/en/galactic/Installation.html
```
2. Install development tools and ROS tools:
```
sudo apt update && sudo apt install -y \
  build-essential \
  cmake \
  git \
  python3-colcon-common-extensions \
  python3-flake8 \
  python3-pip \
  python3-pytest-cov \
  python3-rosdep \
  python3-setuptools \
  python3-vcstool \
  wget
```
3. Install some pip packages needed for testing:
```
python3 -m pip install -U \
  flake8-blind-except \
  flake8-builtins \
  flake8-class-newline \
  flake8-comprehensions \
  flake8-deprecated \
  flake8-docstrings \
  flake8-import-order \
  flake8-quotes \
  pytest-repeat \
  pytest-rerunfailures \
  pytest \
  setuptools
```
4. Make a new ros workspace, for example:
```
mkdir -p item_prep_ws/src
```
5. Go into the `src` folder and clone this repo:
```
cd item_prep_ws/src
git clone https://github.com/endre90/item_preparation.git
```
6. Go back to the woskspace and build it:
```
cd item_prep_ws
colcon build
```
7. For convenience, make a bash (or bat on w10) file for sourcing the workspaces. You can name it, for example: `item_prep_ws.bash`
```
source /opt/ros/install/setup.bash
source ~/item_prep_ws/install/local_setup.bash
```

## What to do next:
1.  Get the item and tool meshes from:
```
https://app.box.com/folder/148049586212
``` 
2. Extract the `preparation` folder to a convenient location.
3. Edit this config file so that it matches your path for the `preparation` folder: 
```
/item_prep_ws/src/bringup/config/paths.json
```

## Try it out:
1. The preparation suite can be launched with the optional arguments:
```
item ->  name of the item to be prepared
         the mesh of the item has to exist
         the .json config file in `parameters` has to exist
tool ->  name of the tool to verify grasping
         RECOMMENDED: use `arrow` for preparation
         later use other tools for verification
         this is because it is more precise to use the `arrow`
scale -> the scale of the controller marker
         for instance, the sponge tool is quite big
         so the controller marker has to be scaled
         in order to smoothly move the mesh around
```
2. Source the workspaces. If you have made the bash file, run:
```
. item_prep_ws.bash
```
3. Launch the preparation suite, for example:
```
ros2 launch bringup preparation.launch.py item:=plug_connector
```

## Actual preparation:
1. Prepare the main frame of the item to be prepared in a CAD software. For example, I like to use Autodesk Inventor since it has easy tools like `flush` and `mate`.
2. Copy and use the `template.json` in `parameters` to make a .json file for a specific item. NOTE: The name of the .json file and the name of the frame have to be the same.
3. Launch the preparation suite with the new item argument.
4. Move the arrow with the controller until you are satisfied with a new secondary position.
5. The window where you have launched the suite from will print the feedback from the controller. This is the current pose of the tool or arrow relative to the main frame that has been prepared in Inventor.
6. Now all that is left to do is copy the transform to the `secondary_transforms` list of its .json file, and that's it.
7. If you would like to see the frame that you have added, just re-build the workspace and launch the suite again. The new frame should be there.
You can always remvoe the frame by removing it from the `secondary_transforms` list and re-building the workspace.