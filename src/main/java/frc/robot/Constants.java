// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static final class DriveConstants {
    // Driving Parameters - Note that these are not the maximum capable speeds of
    // the robot, rather the allowed maximum speeds
    public static final double kMaxSpeedMetersPerSecond = 4.8;
    public static final double kMaxAngularSpeed = 2 * Math.PI; // radians per second

    // Chassis configuration
    public static final double kTrackWidth = Units.inchesToMeters(23);
    // Distance between centers of right and left wheels on robot
    public static final double kWheelBase = Units.inchesToMeters(23);
    // Distance between front and back wheels on robot
    public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
        new Translation2d(kWheelBase / 2, kTrackWidth / 2),
        new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
        new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
        new Translation2d(-kWheelBase / 2, -kTrackWidth / 2));

    // Angular offsets of the modules relative to the chassis in radians
    public static final double kFrontLeftChassisAngularOffset = -Math.PI / 2;
    public static final double kFrontRightChassisAngularOffset = 0;
    public static final double kBackLeftChassisAngularOffset = Math.PI;
    public static final double kBackRightChassisAngularOffset = Math.PI / 2;

    // SPARK MAX CAN IDs
    public static final int kFrontLeftDrivingCanId = 1;
    public static final int kRearLeftDrivingCanId = 5;
    public static final int kFrontRightDrivingCanId = 3;
    public static final int kRearRightDrivingCanId = 7;

    public static final int kFrontLeftTurningCanId = 2;
    public static final int kRearLeftTurningCanId = 6;
    public static final int kFrontRightTurningCanId = 4;
    public static final int kRearRightTurningCanId = 8;

    public static final boolean kGyroReversed = false;
  }

  public static final class ModuleConstants {
    // The MAXSwerve module can be configured with one of three pinion gears: 12T,
    // 13T, or 14T. This changes the drive speed of the module (a pinion gear with
    // more teeth will result in a robot that drives faster).
    public static final int kDrivingMotorPinionTeeth = 14;

    // Calculations required for driving motor conversion factors and feed forward
    public static final double kDrivingMotorFreeSpeedRps = NeoMotorConstants.kFreeSpeedRpm / 60;
    public static final double kWheelDiameterMeters = 0.0762;
    public static final double kWheelCircumferenceMeters = kWheelDiameterMeters * Math.PI;
    // 45 teeth on the wheel's bevel gear, 22 teeth on the first-stage spur gear, 15
    // teeth on the bevel pinion
    public static final double kDrivingMotorReduction = (45.0 * 22) / (kDrivingMotorPinionTeeth * 15);
    public static final double kDriveWheelFreeSpeedRps = (kDrivingMotorFreeSpeedRps * kWheelCircumferenceMeters)
        / kDrivingMotorReduction;
  }

  public static final class OIConstants {
    public static final int kDriverControllerPort = 0;
    public static final double kDriveDeadband = 0.05;
  }

  public static final class VisionConstants {
    public static final String kLimelightTableName = "limelight";

    // Auto-aim turning assist gains/limits.
    public static final double kAimKp = 0.02;
    public static final double kAimToleranceDeg = 1.0;
    public static final double kAimMaxTurnCommand = 0.5;
    public static final double kAimMinTurnCommand = 0.06;
  }

  public static final class AutoConstants {
    public static final double kMaxSpeedMetersPerSecond = 4;
    public static final double kMaxAccelerationMetersPerSecondSquared = 4;
    public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
    public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;

    public static final double kPXController = 1;
    public static final double kPYController = 1;
    public static final double kPThetaController = 1;

    // Constraint for the motion profiled robot angle controller
    public static final TrapezoidProfile.Constraints kThetaControllerConstraints = new TrapezoidProfile.Constraints(
        kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);

    // PathPlanner defaults
    public static final String kDefaultAutoName = "LeftTrenchAuto";
    public static final double kPPTranslationP = 5.0;
    public static final double kPPTranslationI = 0.0;
    public static final double kPPTranslationD = 0.0;
    public static final double kPPRotationP = 5.0;
    public static final double kPPRotationI = 0.0;
    public static final double kPPRotationD = 0.0;
  }

  public static final class IntakeConstants {
    public static final int kLiftMotorCanId = 14;
    public static final int kRollerMotorCanId = 15;

    public static final boolean kLiftMotorInverted = false;

    public static final double kLiftUpSpeed = 0.35;
    public static final double kLiftDownSpeed = 0.6;
    // Startup position is raised/home (zero); start the robot with the intake raised.
    // Negative motor rotations deploy the intake. Tune travel for the current mechanism.
    public static final double kLiftRaisedPositionRotations = 0.0;
    public static final double kLiftDeployedPositionRotations = -9.5;
    // Signed deploy travel; B subtracts this to move back toward home.
    public static final double kLiftRaiseByBRotations = kLiftDeployedPositionRotations;
    public static final double kLiftPositionToleranceRotations = 0.2;
    public static final double kLiftReengageToleranceRotations = 0.6;
    public static final double kLiftPositionKp = 0.08;
    public static final double kLiftMinCommand = 0.06;
    public static final double kLiftMinPositionRotations = kLiftDeployedPositionRotations - 0.2;
    public static final double kLiftMaxPositionRotations = kLiftRaisedPositionRotations;
    public static final double kRollerIntakeSpeed = 0.75;
    public static final double kRollerEjectSpeed = -0.6;
    public static final int kLiftCurrentLimitAmps = 30;
    public static final int kRollerCurrentLimitAmps = 20;
    public static final double kLiftRampRateSeconds = 0.20;
    public static final double kRollerRampRateSeconds = 0.20;
    }

  public static final class ConveyorConstants {
    public static final int kMotorCanId = 10;

    public static final double kConveyorSpeed = 1;
    public static final double kConveyorReverseSpeed = 0.6;
    public static final double kConveyorFeedSpeed = 0.35;
    public static final int kCurrentLimitAmps = 15;
    public static final double kRampRateSeconds = 0.35;
  }

  public static final class FeederConstants {
    public static final int kMotorCanId = 11;

    // If true, the feeder uses the conveyor motor instead of its own motor.
    public static final boolean kUseConveyorMotor = false;

    public static final double kFeederSpeed = 0.5;
    public static final double kFeederReverseSpeed = -0.6;
    public static final double kFeederFeedSpeed = 0.45;
    public static final int kCurrentLimitAmps = 15;
    public static final double kRampRateSeconds = 0.35;
  }

  public static final class ShooterConstants {
    public static final int kLeftMotorCanId = 12;
    public static final int kRightMotorCanId = 13;

    public static final boolean kRightMotorInverted = true;

    public static final double kShooterOpenLoopSpeed = 0.75;
    public static final double kShooterTargetRpm = 3200;
    public static final double kShooterAtSpeedToleranceRpm = 75;
    public static final double kShooterVelocityP = 0.00045;
    public static final double kShooterVelocityI = 0.0;
    public static final double kShooterVelocityD = 0.0;
    public static final double kShooterVelocityKv = 1.0 / NeoMotorConstants.kFreeSpeedRpm;
    public static final int kCurrentLimitAmps = 25;
    public static final double kClosedLoopRampRateSeconds = 0.50;
  }

  public static final class NeoMotorConstants {
    public static final double kFreeSpeedRpm = 5676;
  }
}
