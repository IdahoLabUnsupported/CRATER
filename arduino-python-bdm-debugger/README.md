# Arduino+Python BDM Debugger


## About

This is a simple (and somewhat limited) interface for BDM devices using an Arduino as a pseudo-debugger. The main functions of this program are to dump firmware from a system via debuggin ports, but it has the ability to upload binary files to a device, read/write CPU registers, and step through insturctions like the "step" command in GDB (with limitations.)

This cannot be used as a full-fledged debugger, as in, you cannot give the program source code and step through the program like you can an actual debugger. The BDM protocol is complicated and full debugging features were not necessary during the development.


## Setup

Other than Python and a program to flash the Arduino, there is nothing else to install. This is compatible on both Windows and Linux (tested on Windows 10/11, Ubuntu 24 and Arch). This was not tested on Mac so whether or not it runs on Mac cannot be confirmed.

### Requirements:
An Arduino Uno flashed programmed with the `ard_bdm2.ino` sketch (this could work with other Arduinos, but this is specific to Atmega328 variants)

Wire the Arudino to the BDM header:

[This site has a diagram of the BDM header](https://cmp.felk.cvut.cz/~pisa/m683xx/bdm_driver.html). You can also find a pinout diagram without the Arduino by running the script from the command line then type either `pinout` or `pins`. (If you do not have the Arduino, you can pass `test` when starting the program to skip the serial device initialization. More info in the Usage section).

This is the pin mapping is as follows:

| Arduino Uno Pin number | BDM Pin |
| :--------------------: | :-----: |
| Pin 2 | DSO |
| Pin 3 | DSI |
| Pin 4 | RESET |
| Pin 7 | FREEZE |
| Pin 8 | BERR |
| Pin 12 | DSCLK |



#### Python 3
- `pyserial` - for the connection to the Arduino
- `rich` - for pretty formatting
- `prompt_toolkit` - for a better console interface



## Usage
Open the folder containing `redux_bdmser.py` in the terminal.

Start the program and pass the path to the Serial Arduino device (something like `/dev/ttyACM0` on Linux and `COM3` on Windows). You can start the program without a serial device by passing it the argument `test` (the position does not matter).

Examples:
- To run the program without a serial device: `python ./redux_bdmser.py test`
- To run it normally: `python .\redux_bdmser.py COM3`

From there, a help menu can be displayed by typing `h` or `help`.


## Other notable features

- Pretty formatting using tables
- Setting and clearing CPU flags (Zero, Carry, etc.)
- Detailed help menu and pinout diagram in the command line
- Terminal commands can be executed by starting the line with `#`
  - This can be used to find files with `dir` or `ls` without switching tabs
- Repeat previous commands by pressing `Enter` on a blank line, press `up` and `down` to cycle through command history
- Multiple commands can be run on the same line by separating them with `\`
- The `notes` command lists out warnings and errata that have been documented during development.
