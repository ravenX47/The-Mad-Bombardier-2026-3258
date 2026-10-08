package frc.robot;

import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.config.AbsoluteEncoderConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import frc.robot.Constants.ModuleConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.ConveyorConstants;
import frc.robot.Constants.FeederConstants;

public final class Configs {
    public static final class MAXSwerveModule {
        public static final SparkMaxConfig drivingConfig = new SparkMaxConfig();
        public static final SparkMaxConfig turningConfig = new SparkMaxConfig();

        static {
            // Use module constants to calculate conversion factors and feed forward gain.
            double drivingFactor = ModuleConstants.kWheelDiameterMeters * Math.PI
                    / ModuleConstants.kDrivingMotorReduction;
            double turningFactor = 2 * Math.PI;
            double nominalVoltage = 12.0;
            double drivingVelocityFeedForward = nominalVoltage / ModuleConstants.kDriveWheelFreeSpeedRps;

            drivingConfig
                    .idleMode(IdleMode.kBrake)
                    .smartCurrentLimit(50)
                    .closedLoopRampRate(0.15)
                    .voltageCompensation(12.0);
            drivingConfig.encoder
                    .positionConversionFactor(drivingFactor) // meters
                    .velocityConversionFactor(drivingFactor / 60.0); // meters per second
            drivingConfig.closedLoop
                    .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                    // These are example gains you may need to them for your own robot!
                    .pid(0.04, 0, 0)
                    .outputRange(-1, 1)
                    .feedForward.kV(drivingVelocityFeedForward);

            turningConfig
                    .idleMode(IdleMode.kBrake)
                    .smartCurrentLimit(20)
                    .closedLoopRampRate(0.10)
                    .voltageCompensation(12.0);

            turningConfig.absoluteEncoder
                    // Invert the turning encoder, since the output shaft rotates in the opposite
                    // direction of the steering motor in the MAXSwerve Module.
                    .inverted(true)
                    .positionConversionFactor(turningFactor) // radians
                    .velocityConversionFactor(turningFactor / 60.0) // radians per second
                    // This applies to REV Through Bore Encoder V2 (use REV_ThroughBoreEncoder for V1):
                    .apply(AbsoluteEncoderConfig.Presets.REV_ThroughBoreEncoderV2);

            turningConfig.closedLoop
                    .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
                    // These are example gains you may need to them for your own robot!
                    .pid(1, 0, 0)
                    .outputRange(-1, 1)
                    // Enable PID wrap around for the turning motor. This will allow the PID
                    // controller to go through 0 to get to the setpoint i.e. going from 350 degrees
                    // to 10 degrees will go through 0 rather than the other direction which is a
                    // longer route.
                    .positionWrappingEnabled(true)
                    .positionWrappingInputRange(0, turningFactor);
        }
    }

    public static final class Intake {
        public static final SparkMaxConfig liftConfig = new SparkMaxConfig();
        public static final SparkMaxConfig rollerConfig = new SparkMaxConfig();

        static {
            liftConfig
                    .idleMode(IdleMode.kBrake)
                    .smartCurrentLimit(IntakeConstants.kLiftCurrentLimitAmps)
                    .openLoopRampRate(IntakeConstants.kLiftRampRateSeconds)
                    .voltageCompensation(12.0)
                    .inverted(IntakeConstants.kLiftMotorInverted);

            rollerConfig
                    .idleMode(IdleMode.kCoast)
                    .smartCurrentLimit(IntakeConstants.kRollerCurrentLimitAmps)
                    .openLoopRampRate(IntakeConstants.kRollerRampRateSeconds)
                    .voltageCompensation(12.0);
        }
    }

    public static final class Conveyor {
        public static final SparkMaxConfig conveyorConfig = new SparkMaxConfig();

        static {
            conveyorConfig
                    .idleMode(IdleMode.kCoast)
                    .smartCurrentLimit(ConveyorConstants.kCurrentLimitAmps)
                    .openLoopRampRate(ConveyorConstants.kRampRateSeconds)
                    .voltageCompensation(12.0);
        }
    }

    public static final class Feeder {
        public static final SparkMaxConfig feederConfig = new SparkMaxConfig();

        static {
            feederConfig
                    .idleMode(IdleMode.kCoast)
                    .smartCurrentLimit(FeederConstants.kCurrentLimitAmps)
                    .openLoopRampRate(FeederConstants.kRampRateSeconds)
                    .voltageCompensation(12.0);
        }
    }

    public static final class Shooter {
        public static final SparkMaxConfig leftConfig = new SparkMaxConfig();
        public static final SparkMaxConfig rightConfig = new SparkMaxConfig();

        static {
            leftConfig
                    .idleMode(IdleMode.kCoast)
                    .smartCurrentLimit(ShooterConstants.kCurrentLimitAmps)
                    .closedLoopRampRate(ShooterConstants.kClosedLoopRampRateSeconds)
                    .voltageCompensation(12.0);
            leftConfig.closedLoop
                    .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                    .pid(
                            ShooterConstants.kShooterVelocityP,
                            ShooterConstants.kShooterVelocityI,
                            ShooterConstants.kShooterVelocityD)
                    .outputRange(-1, 1)
                    .feedForward.kV(ShooterConstants.kShooterVelocityKv);

            rightConfig
                    .idleMode(IdleMode.kCoast)
                    .smartCurrentLimit(ShooterConstants.kCurrentLimitAmps)
                    .closedLoopRampRate(ShooterConstants.kClosedLoopRampRateSeconds)
                    .voltageCompensation(12.0)
                    .inverted(ShooterConstants.kRightMotorInverted);
            rightConfig.closedLoop
                    .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                    .pid(
                            ShooterConstants.kShooterVelocityP,
                            ShooterConstants.kShooterVelocityI,
                            ShooterConstants.kShooterVelocityD)
                    .outputRange(-1, 1)
                    .feedForward.kV(ShooterConstants.kShooterVelocityKv);
        }
    }
}
