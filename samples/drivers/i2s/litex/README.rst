.. i2s_litex_example:

#####################
I2S Receiving example
#####################

Overview
********

This is a simple I2S recording example. It allows recording the audio data from I2S input and sending it via UART.

Audio Format
************

The driver provides Audio data with the following parameters:

* 44100 kHz sample rate
* signed 24 bit PCM
* Stereo
* Little endian

Building
********

.. code-block::

   mkdir build && cd build
   cmake -DBOARD=litex_vexriscv ..
   make

Simple test case
****************

The example app will record predefined number of audio samples (256000 by default) and send the data to the host over UART.
In the **tools** directory you can find the **collect_data.py** script.
This script can be used to download the recorded data over UART and write into a file.
The recorded data is a raw PCM data. To play it in Linux you can use e.g. ``aplay`` or ``audacity``.

Known issues
************

When using I2S LiteX driver, the configuration should be delayed 5 seconds (this delay is included in this example).

Notes
*****

* The data has to be read periodically from the driver's buffer, if the application fail to keep up, the data may be lost
* You can configure the size of the audio driver's ring buffer via Kconfig the bigger the buffer, the less often application need to read it
