# roboclaw

Enhanced version of the repository from Bartosz Meglicki (https://github.com/bmegli/roboclaw)

In contrast to his code I implemented only funktions for motor positioning.

It can be used for communication with [Roboclaw](https://www.basicmicro.com//) motor controllers.

The manual can be downloaded here: https://downloads.basicmicro.com/docs/roboclaw_user_manual.pdf

## Platforms 

Library works on Unix platforms (e.g. Linux) It was tested with Microsoft Visual Studio coss development on a Raspberry Pi 3.
The project-file is included.

It should work out the of box with a Raspberry running PiOS. On Ubuntu one has to get rid of the getty-process that by default is using the UART. 

## Hardware

You need to connect Roboclaw (or multiple Roboclaws) via USB or uart

## Why another library

1. Conversion to C++ and separating the serial communication to an extra class.

2. Reading address, device and baudrate from a config-file

3. Setting up the module from faktory reset via a configuration file created by the Basicmicro Motion Studio.

## Scope

The library implements only a subset of commands that I use and I have no plans to implement all possible Roboclaw commands.

Currently implemented:

- `roboclaw_reset`
- `roboclaw_set_encoder_m1`
- `roboclaw_set_encoder_m2`
- `roboclaw_set_pid_m1`
- `roboclaw_set_pid_m2`
- `roboclaw_set_pos_pid_m1`
- `roboclaw_set_pos_pid_m2`
- `roboclaw_speed_accel_decel_pos_m1`
- `roboclaw_speed_accel_decel_pos_m2`

- `roboclaw_read_version`
- `roboclaw_read_encoder`

As Interface the following functions are implemented:

- `startMove`
- `waitCompletion`
- `getPosition`
- `Reset`

- `setSteps`
- `setMicrostep`
- `setTarget`
- `setSpeed`
- `setAcceleration`
- `setDeceleration`
- `setCorridor`
- `setFilter`

## Building Instructions

Follow the build instruction of Visual Studio

### Cloning the repository

``` bash
git clone https://github.com/woko54/roboclaw.git
```

### Building the examples

An example is include in the main-file

## Testing

Run as sudo or add your user to dialout group:

```bash
usermod -a -G dialout your_user
```

Run `RoboclawTest.out` 

| Controller                               | Device (typically)                                                         |
|------------------------------------------|----------------------------------------------------------------------------|
| USB (any controller)                     | `/dev/ttyACM0`                                                             |
| RasberryPi                               | `/dev/ttyAMA0`                                                             |
| BeagleBoneBlack                          | `/dev/ttyO1`, `/dev/ttyO2`, `/dev/ttyO4`                                   |
| LegoMindstorms EV3                       | `/dev/tty_in1`, `/dev/tty_in2`, `/dev/tty_in3`, `/dev/tty_in4`             |

``` bash
./RoboclawTest.out
```

## Using

See main-file.

### Compiling your code

Follow the build instruction of Visual Studio

## License

Library is licensed under Mozilla Public License, v. 2.0.

This is similiar to LGPL but more permissive:
- you can use it as LGPL in prioprietrary software
- unlike LGPL you may compile it statically with your code

Like in LGPL, if you modify this library, you have to make your changes publicly available.
Making a github fork of the library with your changes satisfies those requirements perfectly.
