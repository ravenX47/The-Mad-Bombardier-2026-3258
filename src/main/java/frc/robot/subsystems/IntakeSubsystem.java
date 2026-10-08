package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs;
import frc.robot.Constants.IntakeConstants;

public class IntakeSubsystem extends SubsystemBase {
  private final SparkMax m_liftMotor = new SparkMax(IntakeConstants.kLiftMotorCanId, MotorType.kBrushless);
  private final SparkMax m_rollerMotor = new SparkMax(IntakeConstants.kRollerMotorCanId, MotorType.kBrushless);

  private final RelativeEncoder m_liftEncoder = m_liftMotor.getEncoder();
  private final RelativeEncoder m_rollerEncoder = m_rollerMotor.getEncoder();
  private double m_rollerRequestedOutput = 0.0;
  private double m_liftTargetRotations = IntakeConstants.kLiftRaisedPositionRotations;
  private boolean m_liftHoldingAtTarget = false;

  public IntakeSubsystem() {
    m_liftMotor.configure(Configs.Intake.liftConfig, ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);
    m_rollerMotor.configure(Configs.Intake.rollerConfig, ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);
    // Assumes robot boots with intake raised; this is the position reference for limits.
    setCurrentPositionAsRaisedHome();
  }

  public void setLiftSpeed(double speed) {
    double position = getLiftPositionRotations();
    if ((position <= IntakeConstants.kLiftMinPositionRotations && speed < 0)
        || (position >= IntakeConstants.kLiftMaxPositionRotations && speed > 0)) {
      m_liftMotor.set(0);
      return;
    }
    m_liftMotor.set(speed);
  }

  public void moveLiftToPosition(double targetPositionRotations) {
    double clampedTarget =
        Math.max(
            IntakeConstants.kLiftMinPositionRotations,
            Math.min(IntakeConstants.kLiftMaxPositionRotations, targetPositionRotations));
    if (clampedTarget != m_liftTargetRotations) {
      m_liftHoldingAtTarget = false;
    }
    m_liftTargetRotations = clampedTarget;
    double error = clampedTarget - getLiftPositionRotations();
    double absError = Math.abs(error);

    if (m_liftHoldingAtTarget) {
      if (absError <= IntakeConstants.kLiftReengageToleranceRotations) {
        setLiftSpeed(0);
        return;
      }
      m_liftHoldingAtTarget = false;
    }

    if (absError <= IntakeConstants.kLiftPositionToleranceRotations) {
      m_liftHoldingAtTarget = true;
      setLiftSpeed(0);
      return;
    }

    double command = IntakeConstants.kLiftPositionKp * error;
    if (Math.abs(command) < IntakeConstants.kLiftMinCommand) {
      command = Math.copySign(IntakeConstants.kLiftMinCommand, error);
    }

    if (command > 0) {
      command = Math.min(command, IntakeConstants.kLiftUpSpeed);
    } else {
      command = Math.max(command, -IntakeConstants.kLiftDownSpeed);
    }

    setLiftSpeed(command);
  }

  public void setRollerSpeed(double speed) {
    m_rollerRequestedOutput = speed;
    m_rollerMotor.set(speed);
  }

  public void stop() {
    m_liftMotor.set(0);
    setRollerSpeed(0);
  }

  public void resetLiftEncoder() {
    setCurrentPositionAsRaisedHome();
  }

  /** Call with the intake physically raised. This records home without moving it. */
  public void setCurrentPositionAsRaisedHome() {
    setLiftPositionReference(IntakeConstants.kLiftRaisedPositionRotations);
  }

  public void setCurrentPositionAsDeployedReference() {
    setLiftPositionReference(IntakeConstants.kLiftDeployedPositionRotations);
  }

  private void setLiftPositionReference(double positionRotations) {
    m_liftMotor.stopMotor();
    m_liftEncoder.setPosition(positionRotations);
    m_liftTargetRotations = positionRotations;
    m_liftHoldingAtTarget = false;
  }

  public double getLiftPositionRotations() {
    return m_liftEncoder.getPosition();
  }

  public double getLiftVelocityRpm() {
    return m_liftEncoder.getVelocity();
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Intake/LiftPositionRotations", getLiftPositionRotations());
    SmartDashboard.putNumber("Intake/LiftTargetRotations", m_liftTargetRotations);
    SmartDashboard.putNumber("Intake/RollerRequestedOutput", m_rollerRequestedOutput);
    SmartDashboard.putNumber("Intake/RollerAppliedOutput", m_rollerMotor.getAppliedOutput());
    SmartDashboard.putNumber("Intake/RollerCurrentAmps", m_rollerMotor.getOutputCurrent());
    SmartDashboard.putNumber("Intake/RollerBusVoltage", m_rollerMotor.getBusVoltage());
    SmartDashboard.putNumber("Intake/RollerRPM", m_rollerEncoder.getVelocity());
  }
}
