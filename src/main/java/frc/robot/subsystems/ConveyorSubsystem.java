package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs;
import frc.robot.Constants.ConveyorConstants;

public class ConveyorSubsystem extends SubsystemBase {
  private final SparkMax m_conveyorMotor = new SparkMax(ConveyorConstants.kMotorCanId, MotorType.kBrushless);
  private final RelativeEncoder m_conveyorEncoder = m_conveyorMotor.getEncoder();

  public ConveyorSubsystem() {
    m_conveyorMotor.configure(Configs.Conveyor.conveyorConfig, ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);
  }

  public void setSpeed(double speed) {
    m_conveyorMotor.set(speed);
  }

  public void stop() {
    m_conveyorMotor.set(0);
  }

  public double getVelocityRpm() {
    return m_conveyorEncoder.getVelocity();
  }
}
