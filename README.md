# The Mad Bombardier 2026 — Team 3258

2026 FRC robot code for Team 3258's Mad Bombardier, based on the REV MAXSwerve Java Template v2026.0.

See [the online changelog](https://github.com/REVrobotics/MAXSwerve-Java-Template/blob/main/CHANGELOG.md) for information about updates to the template that may have been released since you created your project.

## Description

Team-specific Java robot code with a REV MAXSwerve drivetrain, intake, conveyor,
feeder, dual-motor shooter, navX gyro, Limelight aiming, and PathPlanner autonomous.
The constants and autonomous paths describe Team 3258's hardware and field setup.

Note that this template is designed for a drivetrain composed of four MAXSwerve Modules, each configured with two SPARKS MAX, a NEO as the driving motor, a NEO 550 as the turning motor, and a REV Through Bore Encoder as the absolute turning encoder. If you are using SPARK Flex for either the drive motor or turning motor, you will need to update the classes accordingly.

To get started, make sure you have calibrated the zero offsets for the absolute encoders in Hardware Client 2 using the `Absolute Encoder` utility under the associated turning SPARK devices.

## Prerequisites

* WPILib 2026 development tools and Java 17. The project pins GradleRIO 2026.2.1.
* SPARK MAX Firmware v26.1.0
* Vendor dependencies are recorded in `vendordeps/`, including REVLib v2026.0.0.

## Build

Clone this repository, then open its folder in WPILib VS Code:

```sh
git clone https://github.com/ravenX47/The-Mad-Bombardier-2026-3258.git
cd The-Mad-Bombardier-2026-3258
```

Build on Windows with `.\gradlew.bat clean build`; on macOS/Linux use
`sh ./gradlew clean build`. Internet access is required for dependencies on the
first build. Building does not deploy to a robot.

The saved team number is **3258** in `.wpilib/wpilib_preferences.json`. Teams
adapting this code must change it before deploying through WPILib VS Code.

## Configuration

It is possible that this project will not work for your robot right out of the box. Various things like the CAN IDs, PIDF gains, chassis configuration, etc. must be determined for your own robot!

These values can be adjusted in the `Configs.java` and `Constants.java` files.

Verify motor directions, CAN IDs, encoder calibration, current limits, mechanism
travel, and PathPlanner robot dimensions before enabling a robot. The intake
uses a relative startup reference, not an automatic homing sensor. Build success
does not establish that a different robot is safe to operate. This repository
currently has no automated robot-behavior tests; hardware validation is required.

## Intake home and travel

Start the robot program with the intake physically raised. The lift's relative encoder
records that starting position as home (0 motor rotations). It does not search for a
home switch or detect the intake's physical angle, and restarting the program records
a new home.

Co-driver controls:

* **A:** deploy to `kLiftDeployedPositionRotations` (currently -9.5 motor rotations from home).
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
The lower software limit is -9.7 rotations; the upper limit is home (0).
Dashboard values `Intake/LiftPositionRotations` and `Intake/LiftTargetRotations` show the
measured position and active target. Verify direction and travel on the robot before
using full deployment.

## PathPlanner autos

PathPlanner autos require GUI-generated files under `src/main/deploy/pathplanner` (including
`settings.json`). If these files are missing, autonomous will be disabled at runtime.

The default selection is `LeftTrenchAuto`. Review all paths and event commands
in the PathPlanner GUI before running autonomous on your field setup.

## Attribution and reuse

This project derives from the REV MAXSwerve Java Template and WPILib examples.
Keep the existing copyright notices and the terms in `LICENSE.txt` and
`WPILib-License.md` when redistributing this source. Vendor libraries have their
own licenses. `CHANGELOG.md` records upstream template changes.
