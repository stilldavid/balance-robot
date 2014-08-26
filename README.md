Self Balancing Robot
====================

In this repository is code that (attempts) to make a robot balance on two wheels.

## Theory of Operation

### PID

The classic control algorithm for an inverted pendulum system like a balancing
robot is the [PID Controller](http://en.wikipedia.org/wiki/PID_controller). The
Wikipedia page does a pretty good job of explaining it, but the gist of it is
that it takes three terms and returns an output that is fed to the motor driver
to achieve balance based on the current error (how far off are we), current
error derivative (how fast are we accumulating error), and the integral of the
error (how much accumulated error do we have over time).

Implementations of this algorithm in this application use an accelerometer or
inertial measurement unit to determine the error. Assuming a balance point of
zero degrees (or 90° from horizontal) the user calculates the P, I and D
values. Each value is then multiplied them each by a tuning coefficient that
controls how much sway (npi) that value has over the output, and the three are
then summed and passed on to a drive system.

---

There are many variations and implementations of PID controllers, and systems
can get complicated rather quickly with multiple cascading PIDs. In my
experience, the more complicated the system the harder it is to tune.

### Enter DPA

One of my heroes is [David P.
Anderson](http://www.geology.smu.edu/dpa-www/dpa.html), who is a professor at
Southern Methodist University. He builds robots and is kind enough to share
them with us all on the Internet!

One of his robots is nBot, a self-balancing robot. He published the control
theory in a text file on the Internet it 2002, and it is one of the simplest
and most effective algorithms I've seen in the space to date so I chose to
implement it here. It's a dual PD controller, and skips the integral term
altogether. I urge you to take a look at the text file - it's far simpler to
understand than most PID papers, and I've included a copy in this repository.

Required Libraries
==================

 * The [SparkFun LSM9DS0 library](https://github.com/sparkfun/LSM9DS0_Breakout)
 * PJRC's excellent [Encoder library](http://www.pjrc.com/teensy/td_libs_Encoder.html)
 * The built-in two-wire libraries

The Balancing Robot
===================

I put together a [large (30" high) balancing
robot](http://instagram.com/p/kvu0mqCQR4/) out of Actobotics parts and some
Yumo encoders. The total cost to build it was quite a lot, but it serves as a
great platform for hacking on balancing algorithms.

At its heart is an Arduino Pro Mega.

There is no wish list for the large balancing robot... yet.

The miniature version of the balbot
===================================

This is the smaller version of the balancing robot that I tried to make for a
cost of $150 or less. It uses a 9DoF single-chip IMU and an inexpensive set of
motors and encoders, as well as a $9 motor driver. The platform is a large
piece of prototyping board, and everything is held together with a bit of tape,
some solder and a healthy amount of jumper wires.

Check out the [SparkFun Wish List](https://www.sparkfun.com/wish_lists/92816)
for a list of parts used on the mini balance bot.

### Front
![Front Image](http://i.imgur.com/w3z3UrS.jpg)

### 2/3rds View
![Front Image](http://i.imgur.com/4agWxGx.jpg)
