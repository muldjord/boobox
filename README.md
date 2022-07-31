# BooBox
A fun Raspberry Pi Pico W project that combines a few pieces of hardware to create an IoT talking skull.

Whenever the motion detector triggers The Pico fetches a sound from the provided TCP server and plays it through the speaker. The servo motor matches the sound data to allowing for a skull or other animatronic to open and close its mouth while the sound plays.

DISCLAIMER! I plan on making a much more thorough explanation on how to built your own BooBox when I get the time. For now, what follows is a quick and dirty walkthrough that should provide all necessary information although in a pretty basic and non-visual way.

Here's an example of a BooBox I made looking like the fiendlishly funny "Murray the Demonic Talking Skull" from the Monkey Island games:

[![](https://img.youtube.com/vi/7ivf-3M5PTc/0.jpg)](https://www.youtube.com/watch?v=7ivf-3M5PTc)

## What you need

### Hardware
* Raspberry Pi Pico W
* Tiny speaker (I use an 8 Ohm, 1 W)
* PIR HC-SR501 motion sensor
* SG90 / Micro Servo 9g servo motor
* LM386 amplifier IC
* Custom print board for LM386 IC and required components for an [amplifier with 200 gain](https://www.ti.com/lit/ds/symlink/lm386.pdf)

DISCLAIMER! The LM386 amplifier is the one thing I don't like about the project currently. I want to change the current PWM audio based signal (which has quite a low quality due to it's sawtooth based origins) to an i2s based digital signal. I am looking into changing this part to a MAX98357A class C amplifier using i2s. This is in the mail, so stay tuned for that change to happen "soon'ish". The LM386 work well for analog audio amplification, but the PWM signal output is not optimal.

#### Electronic schematics
These don't exist yet! I have never done schematics for electronics, so I need to figure out how to do this properly before I can make them. I plan to, and they will be added here when they are ready.

A quick and dirty "schematic" is something like this:
* The PIR sensor is connected to VBUS, GND and GP15
* The servo is connected to VBUS, GND and GP16
* PWM audio signal output is at GP28. It is VERY low without an amplifier though
  * My LM386 amplifier is thefore connected to GP28, VBUS and GND
  * My speaker is connected to the LM386 circuitry

### Software
* The official Pico SDK configured and working (https://github.com/raspberrypi/pico-sdk)
  * Quite easy to get working on a standard Ubuntu machine. Just follow their readme.
* The Qt5 open source SDK
  * On Ubuntu simply run: 'sudo apt-get install qtbase5-dev qtchooser qt5-qmake qtbase5-dev-tools'
* The code from this repository

### 3D models / components
All required 3D models are available in the [3D models](3d_models) subdirectory.

They will also be uploaded to my [Printables.com](https://www.printables.com/social/170442-muldjord/models) page soon. Keep an eye out that!

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
* Compile the pico-client
```
$ cd pico-client
$ mkdir build
$ cd build
$ cmake .. -DPICO_BOARD=pico_w -DWIFI_SSID="YOUR_WIFI_SSID" -DWIFI_PASSWORD="YOUR_WIFI_PASSWORD" -DTCP_SERVER_IP="IP_OF_PC_WHERE_SERVER_IS_RUNNING" ..
$ make
```
* Hold down the 'reprogram' button on the pico and connect the USB cable to the PC, then let go of the button to put the Pico into mass storage mode.
* Drag-n-drop the `pico-client.uf2` file onto the Pico mass storage device.

If the server is running, everything is hooked up correctly and you've entered your wifi information and IP, it should now work.