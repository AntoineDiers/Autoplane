Raspberry Pi configuration
================================

.. image:: raspberry_pi.jpg
    :width: 200

Requirements
--------------------------

- A Windows PC
- A 500Go SD card
- A SD card reader

Configuration
--------------------------

- TODO : install the os
- TODO : install docker

Generate UDEV rules 
__________________________

.. danger::
    TODO : other usb devices

When a USB device is plugged, it is randomly assigned to /dev/ttyUSB0, /dev/ttyUSB1, /dev/ttyUSB2...
UDEV rules allows you to chose the name of the file to which a device will be assigned, we need this so that our drivers can find their devices.

We will configure UDEV rules that will match a device to a file depending on which physical USB port it was plugged into.

.. attention:: 
    This means that the USB devices must be connected into the right USB port or they will not work

- Create the following file : **/etc/udev/rules.d/autoplane.rules**
- Write the following content : 

.. code-block::

    SUBSYSTEM=="tty", KERNELS=="3-2:1.0", SYMLINK+="imu"

- Run the following commands to apply the rules :

.. code-block::

    sudo udevadm control --reload-rules
    sudo udevadm trigger
    
