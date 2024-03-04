Kernel module for the Raspberry Pi to 
record a time of the change value of the GPIO pin.


Install Kernel Headers
----------------------

You need the kernel headers matching your Raspberry Pi's kernel version to compile a module. Install them using:

    $ sudo apt-get install raspberrypi-kernel-headers


Compile the Module
------------------
Run make to compile the module:

    $ make

This will compile the module for the kernel version currently running on your Raspberry Pi.



Install the Module
------------------

To install compiled modile change the GPIOs number in script ./run-rpigpio-rec-driver.sh and run the script:

    $ ./run-rpigpio-rec-driver.sh


