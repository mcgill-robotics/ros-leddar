Leddar ROS Package
==================

[status]: https://dev.mcgillrobotics.com/buildStatus/icon?job=ros-leddar/master
[url]: https://dev.mcgillrobotics.com/job/ros-leddar/job/master
[![status]][url]

This ROS package configures and communicates with LeddarTech devices using
their SDK. **This has only been tested on ROS Indigo, Jade and Kinetic with
their Sensor Evaluation Kit.**

This package was originally based on the work here:
[jpmerc/leddartech](https://github.com/jpmerc/leddartech).

Setting up
----------
You must clone this repository as `leddar` into your `catkin` workspace's
`src` directory:

```bash
roscd
cd src
git clone https://github.com/mcgill-robotics/ros-leddar.git leddar
```

Then, install `libqt5serialport5-dev`. On Ubuntu, you can do so as follows:

```bash
sudo apt-get install -y libqt5serialport5-dev
```

Udev rules
----------
To set up udev rules for your device, simply run:

```bash
sudo usermod -aG plugdev ${USER}
sudo cp 10-leddartech.rules /etc/udev/rules.d/
sudo udevadm trigger
```

You might have to restart for this to take effect.
*Note that these rules might only work for the Sensor Evaluation Kit.*

LeddarTech Driver
-----------------
Register for an account at
[support.leddartech.com](https://support.leddartech.com) and download SDK for
your target platform. Current supported version is v3.2.x.
Extract the archive and move the `*.so` files into a directory of your choice,
usually `/usr/lib`. This folder will be your `${LEDDAR_LIB_DIR}` directory.

Compiling
---------
You **must** compile this package before being able to run it. You can do so
by running:

```bash
export LEDDAR_LIB_DIR=/path/to/driver
catkin_make
```

from the root of your workspace.

Running
-------
To run, simply connect the LeddarTech leddar and launch the package with:

```bash
roslaunch leddar leddar.launch serial:=<serial> type:=<type> frame:=<frame_id> fov:=<fov> range:=<range>
```

`serial`, `type`, `frame`, `fov` and `range`  are run-time ROS launch arguments:
- `serial`: Serial number of the device to connect to, default: only one
connected.
- `type`: Connection type (either: `USB` or `SERIAL`), default: `USB`.
- `frame`: `tf` frame to stamp the messages with, default: `leddar`.
- `fov`: Field of view of the device in degrees, default: 45.0 degrees.
- `range`: Maximum range of the device in meters, default: 50.0 meters.

The `leddar` node will output to the following ROS topic:
- `~scan`: `LaserScan` message. Scan data.

Configuring
-----------
To configure the device, take a look at the parameters defined
in [Scan.cfg](cfg/Scan.cfg).

These paramaters can also be updated on the fly with ROS `dynamic_reconfigure`
as such:

```bash
rosrun rqt_reconfigure rqt_reconfigure
```

Visualizing
-----------
The scan data can be conveniently visualized with `rviz`.

You should be able to see something like this:
![rviz](https://cloud.githubusercontent.com/assets/723610/12699528/8cbbe460-c78c-11e5-801d-e6c24fc7da47.png)
