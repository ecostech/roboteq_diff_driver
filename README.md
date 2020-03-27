# roboteq_diff_driver

ROS driver for the Roboteq SDC21xx family of motor controllers in a differential-drive configuration.

Subscribes to cmd_vel, publishes to odom, and broadcasts odom tf.

Also publishes sensor data to some ancillary topics including roboteq/voltage, roboteq/current, roboteq/energy, and roboteq/temperature.

Does not require any MicroBasic script to operate.

## Usage

Clone to src directory of catkin workspace, then `catkin_make`.

Requires ROS serial package. If not already installed:
```
sudo apt-get install ros-<dist>-serial
```
Sample launch files in roboteq_diff_driver/launch.

## Motor Power Connections

This driver expects that the right motor is connected to channel 1 of the motor controller (M1), and the left motor is connected to channel 2 (M2).

It also expects that when it issues positive speed/power commands to the motor controller, this will translate into forward motion at each motor. In a typical differential drive robot, for forward motion the right motor will need to rotate clockwise and the left motor will need to rotate counter-clockwise. This will generally mean that the right motor will need to be connected as expected with positive motor lead on M1+ and negative motor lead on M1-, and the left motor will need to be connected in reverse with the positive motor lead on M2- and the negative motor lead on M2+.  If either side of the robot is moving in reverese when instructed to move forward, usually just swapping the positive and negative motor leads at the motor controller for that side will correct it.

## TODO

- [x] Make topic names and frames configuration parameters configurable at runtime.
- [x] Make robot configuration parameters configurable at runtime.
- [x] Make motor controller device configuration parameters configurable at runtime.
- [ ] Make miscellaneous motor controller configuration parameters configurable at runtime.
- [ ] Implement dynamically enabled forced-run mode to verify correct motor power connections.
- [ ] Implement dynamically enabled self-test mode to verify correct motor power and encoder connections and configuration.

## Authors

* **Chad Attermann** - *Initial work* - [Ecos Technologies](https://github.com/ecostech)

## License

This project is licensed under the BSD License.

