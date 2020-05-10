.. i2s_litex_example:

#####################
I2S example
#####################

Overview
********

This is a simple I2S audio transceiver example. You can plug any source of music and listen to it.

Audio Format
************

The driver requires and provides Audio data with the following parameters:

* 44100 kHz sample rate
* Signed 24 bit PCM
* Stereo
* Little endian

Building
********

.. code-block::

   mkdir build && cd build
   cmake -DBOARD=litex_vexriscv ..
   make

Known issues
************

When using I2S LiteX driver, the configuration should be delayed 5 seconds (this delay is included in this example).

It seems that after a few minutes some music delay occurs, this is because the sound driver is not able to send data as fast as it receives it.
