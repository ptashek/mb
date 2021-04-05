# WARNING
This code is incomplete, entirely experimental, and it's sole purpose is to demonstrate the capabilities of the Atmega4809 microcontroller, while solving a practical problem.
If you decided to use any part of this code in any way and for any purpose, you are doing so entirely at your own risk, 
and acknowledge no responsibility on my part for any damage or harm to people or property.

# Project description
This project implements a cruise control (tempomat) controller that is supposed to be a drop-in replacement for the 14-pin VDO amplifiers found in many cars of the 1990s, which interfaced with an independent electro-mechanical servo, rather than electronic throttle actuators. An example of such system is the VDO Tempostat 83.601.

An earlier version of this project was tested in a W124 280TE, with passable results.
The biggest challenge is tuning the PID control loops, and the speed control loop in this version is using unoptimized, baseline gains.
The servo control loop has been bench-tuned, and works well enough.

TBC...
