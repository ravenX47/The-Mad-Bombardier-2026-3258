package frc.robot.subsystems;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.VisionConstants;

public class LimelightSubsystem extends SubsystemBase {
  private final NetworkTableEntry m_tvEntry;
  private final NetworkTableEntry m_txEntry;
  private final NetworkTableEntry m_tyEntry;
  private final NetworkTableEntry m_taEntry;

  public LimelightSubsystem() {
    NetworkTable table =
        NetworkTableInstance.getDefault().getTable(VisionConstants.kLimelightTableName);
    m_tvEntry = table.getEntry("tv");
    m_txEntry = table.getEntry("tx");
    m_tyEntry = table.getEntry("ty");
    m_taEntry = table.getEntry("ta");
  }

  public boolean hasTarget() {
    return m_tvEntry.getDouble(0.0) >= 1.0;
  }

  public double getTxDeg() {
    return m_txEntry.getDouble(0.0);
  }

  public double getTyDeg() {
    return m_tyEntry.getDouble(0.0);
  }

  public double getTargetArea() {
    return m_taEntry.getDouble(0.0);
  }

  public double getAimTurnCommand() {
    if (!hasTarget()) {
      return 0.0;
    }

    double txDeg = getTxDeg();
    if (Math.abs(txDeg) <= VisionConstants.kAimToleranceDeg) {
      return 0.0;
    }

    double turnCmd = txDeg * VisionConstants.kAimKp;
    turnCmd =
        Math.copySign(
            Math.max(Math.abs(turnCmd), VisionConstants.kAimMinTurnCommand), turnCmd);
    return MathUtil.clamp(
        turnCmd, -VisionConstants.kAimMaxTurnCommand, VisionConstants.kAimMaxTurnCommand);
  }

  @Override
  public void periodic() {}
}
