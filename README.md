# README

This is the Bristol FSAI simulator developed for the FSAI competition. It currently relies heavily on [EUFS Sim](https://gitlab.com/eufs/eufs_sim).

## Versions
- OS: Ubuntu 20.04 (Focal)
- ROS: ROS 2 Galactic

## Setup

0. Follow the [setup guides](https://bristol-fsai.notion.site/ROS-Workspace-Setup-11c29866e62680b3a193ee29496b3f37?pvs=4) if you need to set up a ROS workspace (there are .devcontainer files included in this repo if you are using dev containers).

1. Navigate to your home folder and clone this repo (this will be inside your virtual machine/container)
   ```
   git clone https://github.com/Formula-Student-AI/core-sim
   ```

2. Install ROS Dependencies for EUFS Sim
   ```
   sudo rosdep install --from-paths $EUFS_MASTER --ignore-src -r -y
   ```

3. Build and Install the ROS Packages
   ```
   colcon build --symlink-install
   ```

4. Source the Overlay Setup Script
   ```
   source install/setup.bash
   ```

5. Launch EUFS Sim
   ```
   ros2 launch eufs_launcher eufs_launcher.launch.py
   ```

If you have any problems check [Known Issues with EUFS sim](https://gitlab.com/eufs/eufs_sim/-/wikis/Getting-Started-Guide#4-known-issues-) first. Contact us ([bristol.fsai@gmail.com](mailto:bristol.fsai@gmail.com)) if you think you've found any bugs.

## How to Conritube

**Create new branch**
   ```
   git checkout change_your_branch_name
   ```

**Make your changes**
   ```
   git add file_name ...
   git commit -m "change_your_commit_message"
   git push
   ```

## Credit
- ijnek: https://github.com/ijnek/ros-devcontainer-template
- EUFS: https://gitlab.com/eufs
