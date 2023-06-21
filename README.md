# adaptive-tactile-force-control
This repository contains the code associated to the submission "Adaptive Tactile Force Control in a Parallel Gripper with Low Positioning Resolution".
Specifically, it contains the module for closed loop control of the Robotiq gripper using the Xela sensors as feedback.

## Requirements
- [hsp-panda/xela-server-ros](https://github.com/hsp-panda/xela-server-ros)
- [hsp-panda/panda_moveit_config](https://github.com/hsp-panda/panda_moveit_config)
- [hsp-panda/robotiq](https://github.com/hsp-panda/robotiq)
- [franka-control](#how-to-install-franka-control)
- [simple_pid](#how-to-install-simple-pid)

### How to install franka control
```console
sudo apt install ros-<your_ros_distro>-franka-control
```

### How to install simple pid
```console
pip install simple_pid
```

### Maintainers
This repository is maintained by:

| | |
|:---:|:---:|
| [<img src="https://github.com/xenvre.png" width="40">](https://github.com/xenvre) | [@xenvre](https://github.com/xenvre) |
