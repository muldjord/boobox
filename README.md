# BooBox
A fun Raspberry Pi Pico W project that combines a few pieces of hardware to create an motion-activated talking skull.

Whenever the motion detector triggers The Pico fetches a sound from the provided TCP server and plays it through the speaker. The jaw matches the audio using a tiny servo that pushes down on a piston that rests on whatever surface the skull is placed on. The skull / Pico W is powered using an ordinary phone charger and a micro USB cable.

I plan on making a more thorough step-by-step explanation on how to built your own BooBox when I get the time. For now, what follows is a quick and dirty walkthrough that should provide all necessary information although in a pretty basic and non-visual way.

_**NOTE!** All sounds added to the BooBox server `data` folder **must** be 11KHz 8 bit unsigned `.wav` files! Due to memory limitations of the Raspberry Pi Pico the size of each sound file must also be less than 192kb. Files bigger than this will play, but will be cut off._

Here's an example of a BooBox I made looking like the fiendlishly funny _Murray the Demonic Talking Skull_ from the Monkey Island games:

[![](https://img.youtube.com/vi/NpiYJFMU0TQ/0.jpg)](https://www.youtube.com/watch?v=NpiYJFMU0TQ)

## What you need

### Hardware
* Raspberry Pi Pico W
* Tiny speaker (I use an 8 Ohm, 1 W)
* PIR HC-SR501 motion sensor
* SG90 / Micro Servo 9g servo motor
* MAX98357A class C I2S amplifier module

#### Electronic schematics
Connect everything exactly as seen here. You'll notice that VSYS and GND has many connections. For the VSYS (+5v) you'll need to share one pin from the Pico W. For the GND connections there are several GND spread out over the GPIO pins of the Pico W. You can use any of those except the one marked _AGND_.
![Connection schematic](schematics/connections_schematic.png)

### Software
* The official Pico SDK configured and working (https://github.com/raspberrypi/pico-sdk)
  * Quite easy to get working on a standard Ubuntu machine. Just follow their readme.
* The Qt5 open source SDK
  * On Ubuntu simply run: 'sudo apt-get install qtbase5-dev qtchooser qt5-qmake qtbase5-dev-tools'
* The code from this repository

### 3D models / components
All required 3D models are available in the [3D models](3d_models) subdirectory.

## Building and running
### Server daemon
```
$ cd server
$ qmake
$ make
```
Put your wav files in the `data` directory. Format MUST be mono 11KHz 8 bit unsigned. Now run the server with:
```
$ ./BooBox
```
### Pico client
* Compile the pico_boobox client
```
$ cd client
$ mkdir build
$ cd build
$ cmake -DPICO_BOARD="pico_w" -DWIFI_SSID="SSID" -DWIFI_PASSWORD="PASS" -DTCP_SERVER_IP="SERVERIP" -DTCP_SERVER_PORT="4242" ..
$ make
```
* Hold down the 'reprogram' button on the Pico W and connect the USB cable to the PC, then let go of the button to put the Pico into mass storage mode.
* Drag-n-drop the `pico_boobox.uf2` file onto the Pico mass storage device.

If the server is running, everything is hooked up correctly and you've entered your wifi information and IP, your BooBox should now come to life!
