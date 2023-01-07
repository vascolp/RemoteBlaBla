# Remote Bla Bla

Remote Bla Bla is a Pybricks (https://pybricks.com) program for a LEGO
__PoweredUp TechnicHub__ and a LEGO __PoweredUp Remote Control__ where you
first define how each device answers to remote control button actions
(config mode) and then you play with your MOC/Set using the defined
configuration (play mode).

The configuration is saved so that you can turn off the hub, change
batteries if needed, go back to your hub and everything is there. Just
play. If the configuration is not what you want, you can go back to
config mode and reconfigure the behavior.

Keep in mind that this is a computer with an input device of 7 buttons
and an output device of just two LEDs. This is a Star Trek like
interface: you don't see what you get (YDSWIG).
However, once you learn it, you will notice that it is simple.
No screens attached!

Here is an example video for configuration mode: [RemoteBlaBla](https://youtu.be/Sl9IE1FV5Xk).

Here is an example video for play mode with Zetros: [RemoteBlaBlaZetros](https://youtu.be/I7dKQUzhrtY).

Here is an example video of 42104 Race Truck RC MOD: [42104 RC](https://youtu.be/E4OErWqwlBo) (instructions on [rebrickable](https://rebrickable.com/mocs/MOC-113882/vascolp/42104-race-truck-rc)).

Users manual in PDF is avaiable [here](RemoteBlaBla.pdf).

Some configuration examples [here](SetsModes.md).


### If you like this work, please consider [donation here](https://www.paypal.com/donate/?business=RSDKYYLUPRHDQ&no_recurring=1&item_name=Please+donate+to+help+me+continue+this+free+work.%0AThank+you%21&currency_code=EUR) to help me creating other free tools.


# Installing Remote Bla Bla

The best way to install Remote Bla Bla is to install pybricks firmware with Remote Bla Bla already integrated.
These instructions explain how to do that.

1. Download Remote Bla Bla firmware from here:
   * Technic hub: [TechnicHub](firmware/pybricks-technichub-v3.1.0_RemoteBlaBla_v1.01_cyan4.zip)
   * City hub: [CityHub](firmware/pybricks-cityhub-v3.1.0_RemoteBlaBla_v1.01.zip)
2. Open `https://code.pybricks.com`
3. Click on `Settings & Help`, if needed to open the list of options (the gear on the left) 
4. Select `Install Pybricks Firmware`
5. On step one, select your hub, open the `Advanced` box below if needed, and drag and drop Remote Bla Bla firmware to the box below
6. Notice that the menu will say `Custom Firmware Selected`, with something like: `v3.1.0 [Remote Bla Bla v1.01]`
7. Proceed to remaining steps as explained in the menus

Thats all!

Now, when you start your hub, it will start Remote Bla Bla.

Notice that only Technic Hubs have a config mode, City Hubs need to be configured with the help of another Technic Hub (not enough memory on City Hub).
Refer to [Remote Bla Bla manual](RemoteBlaBla.pdf) to understand how it works.
If this is your first time with Remote Bla Bla, you really need to read the manual, there is no intuitive interface when all you have is 7 buttons and 2 lights.
But that is the whole point of Remote Bla Bla: being able to use a hub in MOCs without a screen attached.

If you need to return to LEGO firmware, open `https://code.pybricks.com` and select `Restore Official LEGO Firmware`.


# Remote Bla Bla and Pybricks v3.2

Remote Bla Bla does not work with Pybricks 3.2.
This means that if you paste Remote Bla Bla [code](RemoteBlaBla.py) to pybricks code on v3.2 it will not work as expected.
This is because Remote Bla Bla demands the ability to connect, disconnect and reconnect devices to the Hub and some of the
improvements in Pybricks v3.2 removed the ability to do this. Pybricks team is aware of the problem and, hopefully, a future
version will solve this issue. Meanwhile, the Remote Bla Bla firmware provided above is made using Pybricks v3.1.

