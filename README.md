# CS 334 Robot Documentation: Stretch

This repository contains resources to get you started with using the SRC's Stretch 3 robot!

To begin, read these two pieces of official documentation from Hello Robot.

1. [Stretch Docs - Hello Robot!](https://docs.hello-robot.com/0.3/getting_started/hello_robot/) introduces you to the basic power on/off, charging, and teleoperation procedures of the robot.
2. [Stretch Docs - Safety Guide](https://docs.hello-robot.com/0.3/hardware/safety_guide) provides more detailed instruction on important safety procedures to follow when operating the robot.

## [0] General notes
- In case of emergency, you can always press the runstop button (side of the head) to disable the robot.
- The battery life varies, and averages around ~1 hour. Watch the [battery indicator](https://docs.hello-robot.com/0.3/hardware/battery_maintenance_guide_se3/#state-of-battery-charge) to make sure the robot doesn't run out of power while you are working on it.

## [1] Startup procedure

1. Flip the power switch and wait a few seconds before removing the clamp from the lift.
2. The robot will start up with the gamepad teleop program running. If you want to do anything else, you’ll need to kill that process by running `stretch_free_robot_process.py`
3. Before you can move the robot, you’ll need to home it (either through the command line or gamepad).

## [2] Shutdown procedure

1. If the arm is extended, retract it first.
2. Clamp the arm on the lift under the shoulder.
3. Shut down the robot computer (`sudo shutdown -h now` or however else)
4. Turn off the power switch
5. Plug in the robot in the room, and check the [indicator on the charger](https://docs.hello-robot.com/0.3/hardware/battery_maintenance_guide_se3/#noco-genius-10-interface) to make sure it is indeed charging!

## [3] Connecting to the robot

You can connect either through SSH or directly on the robot computer.

### a. SSH

1. Make sure your laptop is on a Stanford network (eduroam is ok).
2. Connect by running `ssh USER@ROBOT_IP` and enter your password when prompted.
    
    
    | ROBOT | IP ADDRESS |
    | --- | --- |
    | **inky** | 171.64.68.11 |

### b. Direct connection

1. Use the monitor, keyboard, and mouse on the rolling cart in the robot room.
    1. *There might be multiple keyboards.. use the one that doesn’t have a built-in trackpad.*
2. Plug in the cart (this should be the thickest cable). It’s not enough to just plug in the monitor!
3. Turn on the monitor (rightmost button) and switch it into VGA input mode (second rightmost button)

**Some notes:**

The robots communicate with the monitor through wireless HDMI dongles. There are transmitters on each robot and one receiver connected to the monitor. If both robots are on, you may need to switch the transmitter you are connected to. To do this:

1. Find the transmitter on the robot that is currently connected (on the back of the base).
2. Press the round button to disconnect the transmitter.
3. Press the button on the transmitter of the other robot to connect it.

The keyboard and mouse also work with multiple devices. The keyboard buttons for switching are labelled with stickers, and you can following the same scheme for switching the mouse (small round button on the bottom).

## [4] Transporting the robot

- If you’re manually moving the robot, make sure there is no program controlling the base (free the robot processes if unsure)
- Note that if you tilt the robot and roll it, the orientation of the head camera may shift
- You can hold the robot room door open by pressing the handicap door button on the inside or outside (from the outside, only after you’ve scanned your ID)

## [5] Extras

It can be helpful to define some aliases for commands you use often. Here are some that you can add to the end of your `.bashrc` 

```bash
alias python='python3'

alias free='stretch_free_robot_process.py'
alias home='stretch_robot_home.py'
alias stow='stretch_robot_stow.py'
alias gamepad='stretch_gamepad_teleop.py'

alias driver='ros2 launch stretch_core stretch_driver.launch.py mode:=navigation'
alias lidar='ros2 launch stretch_core rplidar.launch.py'
alias switch_pad='ros2 service call /switch_to_gamepad_mode std_srvs/srv/Trigger'
alias switch_nav='ros2 service call /switch_to_navigation_mode std_srvs/srv/Trigger'
```