package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs;
import frc.robot.Constants.ShooterConstants;

public class ShooterSubsystem extends SubsystemBase {
  private final SparkMax m_leftMotor = new SparkMax(ShooterConstants.kLeftMotorCanId, MotorType.kBrushless);
  private final SparkMax m_rightMotor = new SparkMax(ShooterConstants.kRightMotorCanId, MotorType.kBrushless);

  private final RelativeEncoder m_leftEncoder = m_leftMotor.getEncoder();
  private final RelativeEncoder m_rightEncoder = m_rightMotor.getEncoder();
  private final SparkClosedLoopController m_leftVelocityController = m_leftMotor.getClosedLoopController();
  private final SparkClosedLoopController m_rightVelocityController = m_rightMotor.getClosedLoopController();

  public ShooterSubsystem() {
    m_leftMotor.configure(Configs.Shooter.leftConfig, ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);
    m_rightMotor.configure(Configs.Shooter.rightConfig, ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);
  }

  public void setSpeed(double speed) {
    m_leftMotor.set(speed);
    m_rightMotor.set(speed);
  }

  public void setTargetRpm(double targetRpm) {
    m_leftVelocityController.setSetpoint(targetRpm, ControlType.kVelocity);
    m_rightVelocityController.setSetpoint(targetRpm, ControlType.kVelocity);
  }

  public void stop() {
    setSpeed(0);
  }

  public double getLeftRpm() {
    return m_leftEncoder.getVelocity();
  }

  public double getRightRpm() {
    return m_rightEncoder.getVelocity();
  }

  public double getAverageRpm() {
    return (getLeftRpm() + getRightRpm()) / 2.0;
  }

  public boolean isAtTargetRpm(double targetRpm) {
    return Math.abs(getAverageRpm() - targetRpm) <= ShooterConstants.kShooterAtSpeedToleranceRpm;
  }
}
