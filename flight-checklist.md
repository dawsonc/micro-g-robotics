# Flight Preparation Checklist

- [ ] Open the `micro-g-robotics` folder in VSCode
- [ ] Re-open the `micro-g-robotics` VSCode workspace in the `Micro-G Robotics`
      Docker Container
- [ ] Navigate into the `ws_micro_g` ROS workspace:

      ```sh
      cd ~/ws_micro_g/
      ```

- [ ] Build and source the ROS workspace:

      ```sh
      colcon build --packages-select-regex micro_g* && source install/setup.bash
      ```

- [ ] Connect the laptop charger
- [ ] Verify that the Realsense USB cable and the robot USB cable are connected
      to the USB hub
- [ ] Connect the USB hub to the laptop
- [ ] Verify that the red LED located on the robot control box is on
- [ ] Connect the stepper motor USB to the laptop
- [ ] Verify that the stepper motor's green and red LEDs turn on and that the
      yellow LED is flashing yellow
- [ ] Turn on the powerstrip
- [ ] Verify that the red LEDs located on the robot's joint are on
- [ ] Open `ticgui`
- [ ] Energize and de-energize the stepper motor
- [ ] Launch the `micro_g` system where `<filename>` should be replaced with
      a filename adhering to the convention `<gravity-type>-<recording #>`:

      ```sh
      ros2 launch micro_g micro_g.launch.yml record_bag:=true bag_filename:=src/micro-g-robotics/bags/<filename>
      ```

## Debug Steps

The following section describes several common errors and the steps needed to
resolve them.

### Resolving Interbotix udev error

Should an error occur regarding Interbotix udev permissions, perform the
following steps on the host system:

1. Copy the following udev rules

   ```sh
   # Place this file in /etc/udev/rules.d/
   # Then reload udev by typing 'udevadm control --reload-rules && udevadm trigger'
   # Sets up rules to give permanent names to devices

   # Allow serial devices to be read by anyone
   KERNEL=="ttyUSB*", MODE:="0666"
   KERNEL=="ttyACM*", MODE:="0666"
   KERNEL=="js*", MODE:="0666"
   KERNEL=="video*", MODE:="0666"

   # OpenCM9.04C board
   SUBSYSTEM=="tty", ATTRS{idVendor}=="fff1", ATTRS{idProduct}=="ff48", ENV{ID_MM_DEVICE_IGNORE}="1", SYMLINK+="ttyDXL"

   # U2D2 board (also sets latency timer to 1ms for faster communication)
   SUBSYSTEM=="tty", ATTRS{idVendor}=="0403", ATTRS{idProduct}=="6014", ENV{ID_MM_DEVICE_IGNORE}="1", ATTR{device/latency_timer}="1", SYMLINK+="ttyDXL"

   # RPLidar
   SUBSYSTEM=="tty", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60", SYMLINK+="rplidar"

   # Kobuki base
   SUBSYSTEM=="tty", ATTRS{idVendor}=="0403", ATTRS{idProduct}=="6001", ATTRS{serial}=="kobuki*", ATTR{device/latency_timer}="1", MODE:="0666", GROUP:="dialout", SYMLINK+="kobuki"

   # LifeCam Cinema
   SUBSYSTEM=="video4linux", ATTRS{idVendor}=="045e", ATTRS{idProduct}=="0812", ATTR{index}=="0", SYMLINK+="lifecam"
   ```

2. Paste the copied rules into the file `/etc/udev/rules.d/99-interbotix-udev.rules`

   ```sh
   sudo nano /etc/udev/rules.d/99-interbotix-udev.rules
   ```

3. Apply the udev rules

   ```sh
   sudo udevadm control --reload-rules && sudo udevadm trigger
   ```

### Resolving RealSense udev error

Should and error regarding RealSense udev permissions occur, perform the
following steps on the host system:

1. Copy the following udev rules

   ```sh
   ##Version=1.1##
   # Device rules for Intel RealSense devices (R200, F200, SR300 LR200, ZR300, D400, L500, T200)
   SUBSYSTEMS=="usb", ATTRS{idVendor}=="8086", ATTRS{idProduct}=="0a80", MODE:="0666", GROUP:="plugdev", RUN+="/usr/local/bin/usb-R200-in_udev"
   SUBSYSTEMS=="usb", ATTRS{idVendor}=="8086", ATTRS{idProduct}=="0a66", MODE:="0666", GROUP:="plugdev"
   SUBSYSTEMS=="usb", ATTRS{idVendor}=="8086", ATTRS{idProduct}=="0aa3", MODE:="0666", GROUP:="plugdev"
   SUBSYSTEMS=="usb", ATTRS{idVendor}=="8086", ATTRS{idProduct}=="0aa2", MODE:="0666", GROUP:="plugdev"
   SUBSYSTEMS=="usb", ATTRS{idVendor}=="8086", ATTRS{idProduct}=="0aa5", MODE:="0666", GROUP:="plugdev"
   SUBSYSTEMS=="usb", ATTRS{idVendor}=="8086", ATTRS{idProduct}=="0abf", MODE:="0666", GROUP:="plugdev"
   SUBSYSTEMS=="usb", ATTRS{idVendor}=="8086", ATTRS{idProduct}=="0acb", MODE:="0666", GROUP:="plugdev"
   SUBSYSTEMS=="usb", ATTRS{idVendor}=="8086", ATTRS{idProduct}=="0ad0", MODE:="0666", GROUP:="plugdev"
   SUBSYSTEMS=="usb", ATTRS{idVendor}=="8086", ATTRS{idProduct}=="04b4", MODE:="0666", GROUP:="plugdev"
   SUBSYSTEMS=="usb", ATTRS{idVendor}=="8086", ATTRS{idProduct}=="0ad1", MODE:="0666", GROUP:="plugdev"
   SUBSYSTEMS=="usb", ATTRS{idVendor}=="8086", ATTRS{idProduct}=="0ad2", MODE:="0666", GROUP:="plugdev"
   SUBSYSTEMS=="usb", ATTRS{idVendor}=="8086", ATTRS{idProduct}=="0ad3", MODE:="0666", GROUP:="plugdev"
   SUBSYSTEMS=="usb", ATTRS{idVendor}=="8086", ATTRS{idProduct}=="0ad4", MODE:="0666", GROUP:="plugdev"
   SUBSYSTEMS=="usb", ATTRS{idVendor}=="8086", ATTRS{idProduct}=="0ad5", MODE:="0666", GROUP:="plugdev"
   SUBSYSTEMS=="usb", ATTRS{idVendor}=="8086", ATTRS{idProduct}=="0ad6", MODE:="0666", GROUP:="plugdev"
   SUBSYSTEMS=="usb", ATTRS{idVendor}=="8086", ATTRS{idProduct}=="0af2", MODE:="0666", GROUP:="plugdev"
   SUBSYSTEMS=="usb", ATTRS{idVendor}=="8086", ATTRS{idProduct}=="0af6", MODE:="0666", GROUP:="plugdev"
   SUBSYSTEMS=="usb", ATTRS{idVendor}=="8086", ATTRS{idProduct}=="0afe", MODE:="0666", GROUP:="plugdev"
   SUBSYSTEMS=="usb", ATTRS{idVendor}=="8086", ATTRS{idProduct}=="0aff", MODE:="0666", GROUP:="plugdev"
   SUBSYSTEMS=="usb", ATTRS{idVendor}=="8086", ATTRS{idProduct}=="0b00", MODE:="0666", GROUP:="plugdev"
   SUBSYSTEMS=="usb", ATTRS{idVendor}=="8086", ATTRS{idProduct}=="0b01", MODE:="0666", GROUP:="plugdev"
   SUBSYSTEMS=="usb", ATTRS{idVendor}=="8086", ATTRS{idProduct}=="0b03", MODE:="0666", GROUP:="plugdev"
   SUBSYSTEMS=="usb", ATTRS{idVendor}=="8086", ATTRS{idProduct}=="0b07", MODE:="0666", GROUP:="plugdev"
   SUBSYSTEMS=="usb", ATTRS{idVendor}=="8086", ATTRS{idProduct}=="0b0c", MODE:="0666", GROUP:="plugdev"
   SUBSYSTEMS=="usb", ATTRS{idVendor}=="8086", ATTRS{idProduct}=="0b0d", MODE:="0666", GROUP:="plugdev"
   SUBSYSTEMS=="usb", ATTRS{idVendor}=="8086", ATTRS{idProduct}=="0b3a", MODE:="0666", GROUP:="plugdev"
   SUBSYSTEMS=="usb", ATTRS{idVendor}=="8086", ATTRS{idProduct}=="0b3d", MODE:="0666", GROUP:="plugdev"
   SUBSYSTEMS=="usb", ATTRS{idVendor}=="8086", ATTRS{idProduct}=="0b48", MODE:="0666", GROUP:="plugdev"
   SUBSYSTEMS=="usb", ATTRS{idVendor}=="8086", ATTRS{idProduct}=="0b49", MODE:="0666", GROUP:="plugdev"
   SUBSYSTEMS=="usb", ATTRS{idVendor}=="8086", ATTRS{idProduct}=="0b4b", MODE:="0666", GROUP:="plugdev"
   SUBSYSTEMS=="usb", ATTRS{idVendor}=="8086", ATTRS{idProduct}=="0b4d", MODE:="0666", GROUP:="plugdev"
   SUBSYSTEMS=="usb", ATTRS{idVendor}=="8086", ATTRS{idProduct}=="0b52", MODE:="0666", GROUP:="plugdev"
   SUBSYSTEMS=="usb", ATTRS{idVendor}=="8086", ATTRS{idProduct}=="0b56", MODE:="0666", GROUP:="plugdev"
   SUBSYSTEMS=="usb", ATTRS{idVendor}=="8086", ATTRS{idProduct}=="0b5b", MODE:="0666", GROUP:="plugdev"
   SUBSYSTEMS=="usb", ATTRS{idVendor}=="8086", ATTRS{idProduct}=="0b5c", MODE:="0666", GROUP:="plugdev"
   SUBSYSTEMS=="usb", ATTRS{idVendor}=="8086", ATTRS{idProduct}=="0b64", MODE:="0666", GROUP:="plugdev"
   SUBSYSTEMS=="usb", ATTRS{idVendor}=="8086", ATTRS{idProduct}=="0b68", MODE:="0666", GROUP:="plugdev"
   SUBSYSTEMS=="usb", ATTRS{idVendor}=="8086", ATTRS{idProduct}=="0b6a", MODE:="0666", GROUP:="plugdev"
   SUBSYSTEMS=="usb", ATTRS{idVendor}=="8086", ATTRS{idProduct}=="0b6b", MODE:="0666", GROUP:="plugdev"

   # Intel RealSense recovery devices (DFU)
   SUBSYSTEMS=="usb", ATTRS{idVendor}=="8086", ATTRS{idProduct}=="0ab3", MODE:="0666", GROUP:="plugdev"
   SUBSYSTEMS=="usb", ATTRS{idVendor}=="8086", ATTRS{idProduct}=="0adb", MODE:="0666", GROUP:="plugdev"
   SUBSYSTEMS=="usb", ATTRS{idVendor}=="8086", ATTRS{idProduct}=="0adc", MODE:="0666", GROUP:="plugdev"
   SUBSYSTEMS=="usb", ATTRS{idVendor}=="8086", ATTRS{idProduct}=="0ade", MODE:="0666", GROUP:="plugdev"
   SUBSYSTEMS=="usb", ATTRS{idVendor}=="8086", ATTRS{idProduct}=="0b55", MODE:="0666", GROUP:="plugdev"

   # Intel RealSense devices (Movidius, T265)
   SUBSYSTEMS=="usb", ENV{DEVTYPE}=="usb_device", ATTRS{idVendor}=="8087", ATTRS{idProduct}=="0af3", MODE="0666", GROUP="plugdev"
   SUBSYSTEMS=="usb", ENV{DEVTYPE}=="usb_device", ATTRS{idVendor}=="8087", ATTRS{idProduct}=="0b37", MODE="0666", GROUP="plugdev"
   SUBSYSTEMS=="usb", ENV{DEVTYPE}=="usb_device", ATTRS{idVendor}=="03e7", ATTRS{idProduct}=="2150", MODE="0666", GROUP="plugdev"

   KERNEL=="iio*", ATTRS{idVendor}=="8086", ATTRS{idProduct}=="0ad5", MODE:="0777", GROUP:="plugdev", RUN+="/bin/sh -c 'chmod -R 0777 /sys/%p'"
   DRIVER=="hid_sensor_custom", ATTRS{idVendor}=="8086", ATTRS{idProduct}=="0ad5", RUN+="/bin/sh -c 'chmod -R 0777 /sys/%p && chmod 0777 /dev/%k'"
   KERNEL=="iio*", ATTRS{idVendor}=="8086", ATTRS{idProduct}=="0af2", MODE:="0777", GROUP:="plugdev", RUN+="/bin/sh -c 'chmod -R 0777 /sys/%p'"
   DRIVER=="hid_sensor*", ATTRS{idVendor}=="8086", ATTRS{idProduct}=="0af2", RUN+="/bin/sh -c 'chmod -R 0777 /sys/%p && chmod 0777 /dev/%k'"
   KERNEL=="iio*", ATTRS{idVendor}=="8086", ATTRS{idProduct}=="0afe", MODE:="0777", GROUP:="plugdev", RUN+="/bin/sh -c 'chmod -R 0777 /sys/%p'"
   DRIVER=="hid_sensor_custom", ATTRS{idVendor}=="8086", ATTRS{idProduct}=="0afe", RUN+="/bin/sh -c 'chmod -R 0777 /sys/%p && chmod 0777 /dev/%k'"
   KERNEL=="iio*", ATTRS{idVendor}=="8086", ATTRS{idProduct}=="0aff", MODE:="0777", GROUP:="plugdev", RUN+="/bin/sh -c 'chmod -R 0777 /sys/%p'"
   DRIVER=="hid_sensor_custom", ATTRS{idVendor}=="8086", ATTRS{idProduct}=="0aff", RUN+="/bin/sh -c 'chmod -R 0777 /sys/%p && chmod 0777 /dev/%k'"
   KERNEL=="iio*", ATTRS{idVendor}=="8086", ATTRS{idProduct}=="0b00", MODE:="0777", GROUP:="plugdev", RUN+="/bin/sh -c 'chmod -R 0777 /sys/%p'"
   DRIVER=="hid_sensor_custom", ATTRS{idVendor}=="8086", ATTRS{idProduct}=="0b00", RUN+="/bin/sh -c 'chmod -R 0777 /sys/%p && chmod 0777 /dev/%k'"
   KERNEL=="iio*", ATTRS{idVendor}=="8086", ATTRS{idProduct}=="0b01", MODE:="0777", GROUP:="plugdev", RUN+="/bin/sh -c 'chmod -R 0777 /sys/%p'"
   DRIVER=="hid_sensor_custom", ATTRS{idVendor}=="8086", ATTRS{idProduct}=="0b01", RUN+="/bin/sh -c 'chmod -R 0777 /sys/%p && chmod 0777 /dev/%k'"
   KERNEL=="iio*", ATTRS{idVendor}=="8086", ATTRS{idProduct}=="0b3a", MODE:="0777", GROUP:="plugdev", RUN+="/bin/sh -c 'chmod -R 0777 /sys/%p'"
   DRIVER=="hid_sensor*", ATTRS{idVendor}=="8086", ATTRS{idProduct}=="0b3a", RUN+="/bin/sh -c ' chmod -R 0777 /sys/%p && chmod 0777 /dev/%k'"
   KERNEL=="iio*", ATTRS{idVendor}=="8086", ATTRS{idProduct}=="0b3d", MODE:="0777", GROUP:="plugdev", RUN+="/bin/sh -c 'chmod -R 0777 /sys/%p'"
   DRIVER=="hid_sensor*", ATTRS{idVendor}=="8086", ATTRS{idProduct}=="0b3d", RUN+="/bin/sh -c ' chmod -R 0777 /sys/%p && chmod 0777 /dev/%k'"
   KERNEL=="iio*", ATTRS{idVendor}=="8086", ATTRS{idProduct}=="0b4b", MODE:="0777", GROUP:="plugdev", RUN+="/bin/sh -c 'chmod -R 0777 /sys/%p'"
   DRIVER=="hid_sensor*", ATTRS{idVendor}=="8086", ATTRS{idProduct}=="0b4b", RUN+="/bin/sh -c ' chmod -R 0777 /sys/%p && chmod 0777 /dev/%k'"
   KERNEL=="iio*", ATTRS{idVendor}=="8086", ATTRS{idProduct}=="0b4d", MODE:="0777", GROUP:="plugdev", RUN+="/bin/sh -c 'chmod -R 0777 /sys/%p'"
   DRIVER=="hid_sensor*", ATTRS{idVendor}=="8086", ATTRS{idProduct}=="0b4d", RUN+="/bin/sh -c ' chmod -R 0777 /sys/%p && chmod 0777 /dev/%k'"
   KERNEL=="iio*", ATTRS{idVendor}=="8086", ATTRS{idProduct}=="0b56", MODE:="0777", GROUP:="plugdev", RUN+="/bin/sh -c 'chmod -R 0777 /sys/%p'"
   DRIVER=="hid_sensor*", ATTRS{idVendor}=="8086", ATTRS{idProduct}=="0b56", RUN+="/bin/sh -c ' chmod -R 0777 /sys/%p && chmod 0777 /dev/%k'"
   KERNEL=="iio*", ATTRS{idVendor}=="8086", ATTRS{idProduct}=="0b5b", MODE:="0777", GROUP:="plugdev", RUN+="/bin/sh -c 'chmod -R 0777 /sys/%p'"
   DRIVER=="hid_sensor*", ATTRS{idVendor}=="8086", ATTRS{idProduct}=="0b5b", RUN+="/bin/sh -c ' chmod -R 0777 /sys/%p && chmod 0777 /dev/%k'"
   KERNEL=="iio*", ATTRS{idVendor}=="8086", ATTRS{idProduct}=="0b5c", MODE:="0777", GROUP:="plugdev", RUN+="/bin/sh -c 'chmod -R 0777 /sys/%p'"
   DRIVER=="hid_sensor*", ATTRS{idVendor}=="8086", ATTRS{idProduct}=="0b5c", RUN+="/bin/sh -c ' chmod -R 0777 /sys/%p && chmod 0777 /dev/%k'"
   KERNEL=="iio*", ATTRS{idVendor}=="8086", ATTRS{idProduct}=="0b64", MODE:="0777", GROUP:="plugdev", RUN+="/bin/sh -c 'chmod -R 0777 /sys/%p'"
   DRIVER=="hid_sensor*", ATTRS{idVendor}=="8086", ATTRS{idProduct}=="0b64", RUN+="/bin/sh -c ' chmod -R 0777 /sys/%p && chmod 0777 /dev/%k'"
   KERNEL=="iio*", ATTRS{idVendor}=="8086", ATTRS{idProduct}=="0b68", MODE:="0777", GROUP:="plugdev", RUN+="/bin/sh -c 'chmod -R 0777 /sys/%p'"
   DRIVER=="hid_sensor*", ATTRS{idVendor}=="8086", ATTRS{idProduct}=="0b68", RUN+="/bin/sh -c ' chmod -R 0777 /sys/%p && chmod 0777 /dev/%k'"
   KERNEL=="iio*", ATTRS{idVendor}=="8086", ATTRS{idProduct}=="0b6a", MODE:="0777", GROUP:="plugdev", RUN+="/bin/sh -c 'chmod -R 0777 /sys/%p'"
   DRIVER=="hid_sensor*", ATTRS{idVendor}=="8086", ATTRS{idProduct}=="0b6a", RUN+="/bin/sh -c ' chmod -R 0777 /sys/%p && chmod 0777 /dev/%k'"
   KERNEL=="iio*", ATTRS{idVendor}=="8086", ATTRS{idProduct}=="0b6b", MODE:="0777", GROUP:="plugdev", RUN+="/bin/sh -c 'chmod -R 0777 /sys/%p'"
   DRIVER=="hid_sensor*", ATTRS{idVendor}=="8086", ATTRS{idProduct}=="0b6b", RUN+="/bin/sh -c ' chmod -R 0777 /sys/%p && chmod 0777 /dev/%k'"

   # For products with motion_module, if (kernels is 4.15 and up) and (device name is "accel_3d") wait, in another process, until (enable flag is set to 1 or 200 mSec passed) and then set it to 0.
   KERNEL=="iio*", ATTRS{idVendor}=="8086", ATTRS{idProduct}=="0ad5|0afe|0aff|0b00|0b01|0b3a|0b3d|0b56|0b5c|0b64|0b68|0b6a|0b6b", RUN+="/bin/sh -c '(major=`uname -r | cut -d \".\" -f1` && minor=`uname -r | cut -d \".\" -f2` && (([ $$major -eq 4 ] && [ $$minor -ge 15 ]) || [ $$major -ge 5 ])) && (enamefile=/sys/%p/name && [ `cat $$enamefile` = \"accel_3d\" ]) && enfile=/sys/%p/buffer/enable && echo \"COUNTER=0; while [ \$$COUNTER -lt 20 ] && grep -q 0 $$enfile; do sleep 0.01; COUNTER=\$$((COUNTER+1)); done && echo 0 > $$enfile\" | at now'"
   ```

2. Paste the copied rules into the file `/etc/udev/rules.d/99-realsense-libusb.rules`

   ```sh
   sudo nano /etc/udev/rules.d/99-realsense-libusb.rules
   ```

3. Apply the udev rules

   ```sh
   sudo udevadm control --reload-rules && sudo udevadm trigger
   ```
