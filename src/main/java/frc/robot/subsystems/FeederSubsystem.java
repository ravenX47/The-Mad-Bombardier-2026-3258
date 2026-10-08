package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs;
import frc.robot.Constants.FeederConstants;

public class FeederSubsystem extends SubsystemBase {
  private final ConveyorSubsystem m_conveyorSubsystem;
  private final SparkMax m_feederMotor;
  private final RelativeEncoder m_feederEncoder;

  public FeederSubsystem(ConveyorSubsystem conveyorSubsystem) {
    m_conveyorSubsystem = conveyorSubsystem;

    if (FeederConstants.kUseConveyorMotor) {
      m_feederMotor = null;
      m_feederEncoder = null;
    } else {
      m_feederMotor = new SparkMax(FeederConstants.kMotorCanId, MotorType.kBrushless);
      m_feederEncoder = m_feederMotor.getEncoder();
      m_feederMotor.configure(Configs.Feeder.feederConfig, ResetMode.kResetSafeParameters,
          PersistMode.kPersistParameters);
    }
  }

  public void setSpeed(double speed) {
    if (FeederConstants.kUseConveyorMotor) {
      m_conveyorSubsystem.setSpeed(speed);
      return;
    }

    m_feederMotor.set(speed);
  }

  public void stop() {
    setSpeed(0);
  }

  public double getVelocityRpm() {
    if (FeederConstants.kUseConveyorMotor) {
      return m_conveyorSubsystem.getVelocityRpm();
    }

    return m_feederEncoder.getVelocity();
  }
}
