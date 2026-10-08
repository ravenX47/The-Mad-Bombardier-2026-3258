# The Mad Bombardier 2026 — Team 3258

2026 FRC robot code for Team 3258's Mad Bombardier, based on the REV MAXSwerve Java Template v2026.0.

See [the online changelog](https://github.com/REVrobotics/MAXSwerve-Java-Template/blob/main/CHANGELOG.md) for information about updates to the template that may have been released since you created your project.

## Description

A template project for an FRC swerve drivetrain that uses REV MAXSwerve Modules.

Note that this template is designed for a drivetrain composed of four MAXSwerve Modules, each configured with two SPARKS MAX, a NEO as the driving motor, a NEO 550 as the turning motor, and a REV Through Bore Encoder as the absolute turning encoder. If you are using SPARK Flex for either the drive motor or turning motor, you will need to update the classes accordingly.

To get started, make sure you have calibrated the zero offsets for the absolute encoders in Hardware Client 2 using the `Absolute Encoder` utility under the associated turning SPARK devices.

## Prerequisites

* SPARK MAX Firmware v26.1.0
* REVLib v2026.0.0

## Configuration

It is possible that this project will not work for your robot right out of the box. Various things like the CAN IDs, PIDF gains, chassis configuration, etc. must be determined for your own robot!

These values can be adjusted in the `Configs.java` and `Constants.java` files.

## Intake home and travel

Start the robot program with the intake physically raised. The lift's relative encoder
records that starting position as home (0 motor rotations). It does not search for a
home switch or detect the intake's physical angle, and restarting the program records
a new home.

Co-driver controls:

* **A:** deploy to `kLiftDeployedPositionRotations` (currently -2 motor rotations from home).
* **Y:** return to raised/home (0).
* **B:** raise by `kLiftRaiseByBRotations`, clamped to the travel limits.
* **Back:** record the current position as raised/home and reset the target to home.
  This also works while disabled: physically raise the intake, then press Back.
* **Start:** record the current position as the deployed reference and set the target
  there. Use only when physically at the configured deployed endpoint; this does not
  measure or change the travel distance. This also works while disabled.
* **Left bumper (hold):** run the roller while continuing lift position control.

For changed gearing or linkage, tune `kLiftDeployedPositionRotations` in `Constants.java`
to the measured motor travel. Negative encoder travel deploys the intake.
The lower software limit is -2.2 rotations; the upper limit is home (0).
Dashboard values `Intake/LiftPositionRotations` and `Intake/LiftTargetRotations` show the
measured position and active target. Verify direction and travel on the robot before
using full deployment.

## PathPlanner autos

PathPlanner autos require GUI-generated files under `src/main/deploy/pathplanner` (including
`settings.json`). If these files are missing, autonomous will be disabled at runtime.
