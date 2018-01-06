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

## TODO

- [x] Make topic names and frames configuration parameters configurable at runtime.
- [x] Make robot configuration parameters configurable at runtime.
- [x] Make motor controller device configuration parameters configurable at runtime.
- [ ] Make miscellaneous motor controller configuration parameters configurable at runtime.

## Authors

* **Chad Attermann** - *Initial work* - [Ecos Technologies](https://github.com/ecostech)

## License

This project is licensed under the BSD License.

