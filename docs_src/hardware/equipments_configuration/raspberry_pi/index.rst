Raspberry Pi configuration
================================

.. image:: raspberry_pi.jpg
    :width: 200

Requirements
--------------------------

- A Windows PC
- A 500Go external SSD drive

Configuration
--------------------------

- TODO : install the os
- TODO : install docker

Install and setup OpenVPN 
__________________________

- Install OpenVPN Client

.. code-block::

    sudo apt update
    sudo apt install openvpn

- Setup autostart

Uncomment the following line from **/etc/default/openvpn**

.. code-block::

    AUTOSTART="all"

- Download your .ovpn file from your OpenVpnAS instance on your Dev PC
- scp it in your Raspberry Pi under **/etc/openvpn/client.conf**

- Enable and start the OpenVPN service

.. code-block::

    sudo systemctl enable openvpn@client.service
    sudo systemctl start openvpn@client.service

Generate UDEV rules 
__________________________

When a USB device is plugged, it can be assigned to /dev/ttyUSB0, /dev/ttyUSB1, /dev/ttyUSB2...
UDEV rules allows you to chose the name of the file to which a device will be assigned, we need this so that our drivers can find their devices.

- Create the following file : **/etc/udev/rules.d/autoplane.rules**
- Write the following content : 

.. code-block::

    KERNELS=="ttyUSB*", SYMLINK+="imu"
    KERNELS=="ttyACM*", SYMLINK+="gps"

- Run the following commands to apply the rules :

.. code-block::

    sudo udevadm control --reload-rules
    sudo udevadm trigger
    
