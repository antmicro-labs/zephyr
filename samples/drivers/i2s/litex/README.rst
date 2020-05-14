.. i2s_litex_example:

#####################
I2S Receiving example
#####################

Overview
********
Simple i2s example. You can record the required amount of time of any sound you like and then
send it back to any device via UART.

Tips
*******
* Avoid highly consuming operations during receiving data. 
* You can configure the size of the ring buffer via kconfig.

Format description
******************
Data stored in driver buffer is continuous and composed of the mono 16-bit data samples. Only left channel is stored.

Building
********
.. code-block::

   cmake -DBOARD=litex_vexriscv ..   
   make

Usage
******
* Connect necessary devices to receive data via i2s.
* Send some audio data and run this example on your device.
* Use script **collect_data.py** which you can find in **tools** directory.
* You should configure the number of frames that you want to receive for both collect_data.py and i2s example. The default value is 1000 frames.
* Frame is composed of predefined number of samples. Number of samples is calcualted as **number_of_samples_in_frame * number_of_channels**. Default frame configuration contains 128 samples.
* Type **collect_data.py** to start collecting sound. Received data should be visible in file **sound.raw**
* Data inside sound.raw file is stored as 16-bit PCM data format, little-endian, Mono.
* You can use **sox -t raw -r XXXXX -b 16 -c 1 -L -e signed-integer sound.raw recording.wav** command to convert raw data to wave audio format.
* **XXXXX** should be substituted with received sound frequency eg. 16000
