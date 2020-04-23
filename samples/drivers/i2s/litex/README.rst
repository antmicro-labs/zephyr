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
* Avoid highly consuming operations during receiving data. This may cause data loss.
* You can configure the size of the ring buffer via kconfig. This may delay data loss a bit.

Format description
******************
Data stored in driver buffer is continuous and a single sample is composed of the two 24-bit data channels(L+R).

Building
********
* cmake -DBOARD=litex_vexriscv ..
* make

Simple test case
****************
* Connect necessary devices to receive data via i2s.
* To be sure that data is correctly obtained you should generate sin wave and transmit it to a device.
* The simplest way which I found to generate sin wave was using **pactl load-module module-sine frequency=1000**
* To turn it off type **pactl unload-module module-sine**
* Use script **collect_data.py** which you can find in **tools** directory.
* You should configure the number of frames that you want to receive for both collect_data.py and i2s example. The default value is 1000 frames.
* Type **collect_data.py** to start collecting sound. Received data should be visible in file **sin.raw**
* You can load it to any sound program which allows playing raw sound format. eg. Audacity
* Data inside sin.raw file is stored as 24-bit PCM data format, little-endian, Stereo(2 channels). You should configure app with such setting when loading data, so Audacity or any other program will be able to properly decode signal.
* As a result, you should see sin wave for both channels without any disruption.

Known bugs
**********
When using i2s litex driver, the configuration should be delayed 5 seconds. This is caused by something, but I haven't detected the source of this problem.
