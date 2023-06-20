# Zoomie

This is my entry into the [DIY Robocars race in Oakland CA](https://meetup.com/diyrobocars).

Zoomie is a self-driving racing robot car that aims to complete the circuit as quickly as possible. The goal is to control the car well enough to maximize the friction offered by the tires.

## Hardware

My car is a 3d printed 1/10 scale touring car chassis:

- [Chassis STL files](https://cults3d.com/en/3d-model/various/veltro-1-10-rally-touring-car-chassis)
- Motor: Turnigy 13.5T Sensored Brushless Motor
- ESC: Hobbyking 120A Brushless ESC
- Servo: JX PDI-6208MG (eBay)

Electronics / Sensors:

- Andy's custom [HAT board](https://easyeda.com/a1k0n/cycloid)
  - Odometry via intercepting the motor sensor wires (Andy is brilliant!)
  - Onboard IMU
  - PWM output for ESC and servo
- Raspberry Pi 3
- Arducam Fisheye Camera for Raspberry Pi with M12 Lens, 5MP OV5647 1080P

## Software

The optimal racing line is planned using TUM - Institute of Automotive Technology's race tracjectory optimization tool. I have a custom fork + branch [here](https://github.com/danchia/global_racetrajectory_optimization/tree/zoomie) where I made some tweaks to get the solver to work for the smaller scales of Zoomie.

Other offline tools used for camera calibration etc are stored in `tools/`.

The main driver control loop is found at `src/driver/driver.cc`:

- A 100Hz loop does the follow each cycle
  - Collector odometry and gyroscope reading and perform odometry integration.
  - Add in any vision-based localization corrections.
  - A high-level planner determines the desired forward and angular velocity.
  - A PID controller calculates the ESC, servo output based on the desired velocities.

The camera vision based particle filter operates at 30Hz:

- Applies any odometry / gyro sensor readings.
- Extracts the ceiling light centers and uses that for particle likelihood.
- Produces a correction factor for the main control loop.

The high-level motion planned is based around the Stanford Stanley steering controller, with an additional feedforward component as well as an addition PID control to more gradually move the desired angular velocity toward's Stanley's target velocity. Otherwise I found that the car would oscillate at high speeds from over-correcting.

Telemetry is logged using the [MCAP](https://mcap.dev/) format and visualized using the most excellent [Foxglove Studio](https://foxglove.dev/). I wrote a blog about this for them [here](https://foxglove.dev/blog/recording-robocar-data-with-mcap).

## Acknowledgements

I've been heavily inspired by Andy Sloane [robot car]](https://github.com/a1k0n/cycloid), and part of my impetus for releasing my code is continue the knowledge sharing!
