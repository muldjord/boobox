# BooBox
A fun Raspberry Pi Pico W project that combines a few pieces of hardware to create an IoT talking skull.

Whenever the motion detector triggers The Pico fetches a sound from the provided TCP server and plays it through the speaker. The servo motor matches the sound data to allowing for a skull or other animatronic to open and close its mouth while the sound plays.

## What you need

### Hardware
* Raspberry Pi Pico W
* Tiny speaker (I use an 8 Ohm, 1 W)
* PIR HC-SR501 motion sensor
* SG90 / Micro Servo 9g servo motor
* LM386 amplifier IC
* Custom print board for LM386 IC and required components for an amplifier with 200 gain

DISCLAIMER! The LM386 amplifier is the one thing I don't like about the project currently. It works just fine, but it uses PWM audio, which has quite a low quality due to it's sawtooth based origins. I am looking into changing this part to a MAX98357A class C amplifier using i2s. This is in the mail, so stay tuned for that change to happen "soon'ish".

### Software
* The official Pico SDK configured and working (https://github.com/raspberrypi/pico-sdk)
 * Quite easy to get working on a standard Ubuntu machine. Just follow their readme.
* The Qt5 open source SDK
 * On Ubuntu simply run: 'sudo apt-get install qtbase5-dev qtchooser qt5-qmake qtbase5-dev-tools'
* The code from this repository

### 3D models / components
The 3D models and components needed to make your own Murray, will be uploaded to printables.com soon. Keep an eye out at: https://www.printables.com/social/170442-muldjord/models

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
* hold down the 'reprogram' button on the pico and connect the USB cable to the PC, then let go of the button to put the Pico into mass storage mode.
* Drag-n-drop the `pico-client.uf2` file onto the Pico mass storage device.

If the server is running, everything is hooked up correctly and you've entered your wifi information and IP, it should now work.