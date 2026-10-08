 // Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.PS4Controller.Button;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.ConveyorSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems
  private final DriveSubsystem m_robotDrive = new DriveSubsystem();
  private final LimelightSubsystem m_limelightSubsystem = new LimelightSubsystem();
  private final IntakeSubsystem m_intakeSubsystem = new IntakeSubsystem();
  private final ConveyorSubsystem m_conveyorSubsystem = new ConveyorSubsystem();
  private final FeederSubsystem m_feederSubsystem = new FeederSubsystem(m_conveyorSubsystem);
  private final ShooterSubsystem m_shooterSubsystem = new ShooterSubsystem();

  // The driver's controller
  XboxController m_driverController = new XboxController(OIConstants.kDriverControllerPort);
  XboxController m_codriverController = new XboxController(1);

  private double m_intakeLiftTargetRotations = Constants.IntakeConstants.kLiftRaisedPositionRotations;
  private boolean m_autoIntakeRollerEnabled = false;
  private boolean m_shootingActive = false;
  private Command m_autoShootLoopCommand;
  private final SendableChooser<Command> m_autoChooser;

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    registerPathPlannerNamedCommands();
    m_autoChooser = AutoBuilder.buildAutoChooser(AutoConstants.kDefaultAutoName);
    SmartDashboard.putData("Auto Chooser", m_autoChooser);

    // Configure the button bindings
    configureButtonBindings();

    // Configure default commands
    m_robotDrive.setDefaultCommand(
        // The left stick controls translation of the robot.
        // Turning is controlled by the X axis of the right stick.
        new RunCommand(
            () -> m_robotDrive.drive(
                -MathUtil.applyDeadband(m_driverController.getLeftY(), OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(m_driverController.getLeftX(), OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(m_driverController.getRightX(), OIConstants.kDriveDeadband),
                true),
            m_robotDrive));

    m_intakeSubsystem.setDefaultCommand(
        new RunCommand(
            () -> {
              double rollerSpeed =
                  (m_autoIntakeRollerEnabled && DriverStation.isAutonomousEnabled())
                      ? Constants.IntakeConstants.kRollerIntakeSpeed
                      : 0.0;
              m_intakeSubsystem.moveLiftToPosition(m_intakeLiftTargetRotations);
              m_intakeSubsystem.setRollerSpeed(rollerSpeed);
            },
            m_intakeSubsystem));
  }

  private void registerPathPlannerNamedCommands() {
    NamedCommands.registerCommand(
        "IntakeOn",
        new InstantCommand(
            () -> {
              m_intakeLiftTargetRotations = Constants.IntakeConstants.kLiftDeployedPositionRotations;
              m_autoIntakeRollerEnabled = true;
              m_intakeSubsystem.setRollerSpeed(Constants.IntakeConstants.kRollerIntakeSpeed);
            },
            m_intakeSubsystem));

    NamedCommands.registerCommand(
        "IntakeOff",
        new InstantCommand(
            () -> {
              m_intakeLiftTargetRotations = Constants.IntakeConstants.kLiftRaisedPositionRotations;
              m_autoIntakeRollerEnabled = false;
              m_intakeSubsystem.setRollerSpeed(0.0);
            },
            m_intakeSubsystem));

    NamedCommands.registerCommand(
        "Shoot Start",
        new InstantCommand(
            () -> {
              if (m_autoShootLoopCommand != null) {
                m_autoShootLoopCommand.cancel();
              }

              m_autoShootLoopCommand = new RunCommand(
                  () -> {
                    if (!m_shootingActive) {
                      m_shootingActive = true;
                    }

                    m_shooterSubsystem.setTargetRpm(Constants.ShooterConstants.kShooterTargetRpm);
                    if (m_shooterSubsystem.isAtTargetRpm(Constants.ShooterConstants.kShooterTargetRpm)) {
                      m_conveyorSubsystem.setSpeed(Constants.ConveyorConstants.kConveyorFeedSpeed);
                      m_feederSubsystem.setSpeed(Constants.FeederConstants.kFeederFeedSpeed);
                    } else {
                      m_conveyorSubsystem.stop();
                      m_feederSubsystem.stop();
                    }
                  },
                  m_shooterSubsystem,
                  m_conveyorSubsystem,
                  m_feederSubsystem);

              CommandScheduler.getInstance().schedule(m_autoShootLoopCommand);
            }));

    NamedCommands.registerCommand(
        "Shoot Stop",
        new InstantCommand(
            () -> {
              if (m_autoShootLoopCommand != null) {
                m_autoShootLoopCommand.cancel();
                m_autoShootLoopCommand = null;
              }
              m_shooterSubsystem.stop();
              m_conveyorSubsystem.stop();
              m_feederSubsystem.stop();
              m_shootingActive = false;
            },
            m_shooterSubsystem,
            m_conveyorSubsystem,
            m_feederSubsystem));

  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by
   * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its
   * subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling
   * passing it to a
   * {@link JoystickButton}.
   */
  private void configureButtonBindings() {
    new JoystickButton(m_driverController, Button.kR1.value)
        .whileTrue(new RunCommand(
            () -> m_robotDrive.setX(),
            m_robotDrive));

    new JoystickButton(m_driverController, XboxController.Button.kLeftBumper.value)
        .whileTrue(new RunCommand(
            () -> {
              double xSpeed =
                  -MathUtil.applyDeadband(m_driverController.getLeftY(), OIConstants.kDriveDeadband);
              double ySpeed =
                  -MathUtil.applyDeadband(m_driverController.getLeftX(), OIConstants.kDriveDeadband);
              double manualRot =
                  -MathUtil.applyDeadband(m_driverController.getRightX(), OIConstants.kDriveDeadband);
              double rotCmd = m_limelightSubsystem.hasTarget()
                  ? m_limelightSubsystem.getAimTurnCommand()
                  : manualRot;
              m_robotDrive.drive(xSpeed, ySpeed, rotCmd, true);
            },
            m_robotDrive));

    new JoystickButton(m_driverController, XboxController.Button.kStart.value)
        .onTrue(new InstantCommand(
            () -> m_robotDrive.zeroHeading(),
            m_robotDrive));

    new JoystickButton(m_codriverController, XboxController.Button.kA.value)
        .onTrue(new InstantCommand(
            () -> m_intakeLiftTargetRotations = Constants.IntakeConstants.kLiftDeployedPositionRotations));

    new JoystickButton(m_codriverController, XboxController.Button.kY.value)
        .onTrue(new InstantCommand(
            () -> m_intakeLiftTargetRotations = Constants.IntakeConstants.kLiftRaisedPositionRotations));

    new JoystickButton(m_codriverController, XboxController.Button.kB.value)
        .onTrue(new InstantCommand(
            () -> m_intakeLiftTargetRotations =
                m_intakeSubsystem.getLiftPositionRotations() - Constants.IntakeConstants.kLiftRaiseByBRotations));

    new JoystickButton(m_driverController, XboxController.Button.kB.value)
        .onTrue(new InstantCommand(
            () -> m_intakeLiftTargetRotations =
                m_intakeSubsystem.getLiftPositionRotations() - Constants.IntakeConstants.kLiftRaiseByBRotations));

    new JoystickButton(m_codriverController, XboxController.Button.kBack.value)
        .onTrue(new InstantCommand(
            () -> {
              m_intakeSubsystem.setCurrentPositionAsRaisedHome();
              m_intakeLiftTargetRotations = Constants.IntakeConstants.kLiftRaisedPositionRotations;
            },
            m_intakeSubsystem).ignoringDisable(true));

    new JoystickButton(m_codriverController, XboxController.Button.kStart.value)
        .onTrue(new InstantCommand(
            () -> {
              m_intakeSubsystem.setCurrentPositionAsDeployedReference();
              m_intakeLiftTargetRotations = Constants.IntakeConstants.kLiftDeployedPositionRotations;
            },
            m_intakeSubsystem).ignoringDisable(true));

    new JoystickButton(m_codriverController, XboxController.Button.kLeftBumper.value)
        .whileTrue(new RunCommand(
            () -> {
              m_intakeSubsystem.moveLiftToPosition(m_intakeLiftTargetRotations);
              m_intakeSubsystem.setRollerSpeed(Constants.IntakeConstants.kRollerIntakeSpeed);
            },
            m_intakeSubsystem).finallyDo(() -> m_intakeSubsystem.stop()));


    new JoystickButton(m_codriverController, XboxController.Button.kX.value)
        .whileTrue(new StartEndCommand(
            () -> m_conveyorSubsystem.setSpeed(Constants.ConveyorConstants.kConveyorReverseSpeed),
            () -> m_conveyorSubsystem.stop(),
            m_conveyorSubsystem));
    new JoystickButton(m_codriverController, XboxController.Button.kRightBumper.value)
        .whileTrue(new RunCommand(
            () -> {
              if (!m_shootingActive) {
                m_shootingActive = true;
              }

              m_shooterSubsystem.setTargetRpm(Constants.ShooterConstants.kShooterTargetRpm);
              if (m_shooterSubsystem.isAtTargetRpm(Constants.ShooterConstants.kShooterTargetRpm)) {
                m_conveyorSubsystem.setSpeed(Constants.ConveyorConstants.kConveyorFeedSpeed);
                m_feederSubsystem.setSpeed(Constants.FeederConstants.kFeederFeedSpeed);
              } else {
                m_conveyorSubsystem.stop();
                m_feederSubsystem.stop();
              }
            },
            m_shooterSubsystem,
            m_conveyorSubsystem,
            m_feederSubsystem))
        .onFalse(new InstantCommand(
            () -> {
              m_shooterSubsystem.stop();
              m_conveyorSubsystem.stop();
              m_feederSubsystem.stop();
              m_shootingActive = false;
            },
            m_shooterSubsystem,
            m_conveyorSubsystem,
            m_feederSubsystem));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    if (m_autoChooser == null) {
      DriverStation.reportWarning("Auto chooser unavailable; autonomous disabled.", false);
      return null;
    }

    Command selectedAuto = m_autoChooser.getSelected();
    if (selectedAuto == null) {
      DriverStation.reportWarning("No auto selected in chooser; autonomous disabled.", false);
      return null;
    }

    return selectedAuto;
  }

  public double getHeadingDeg() {
    return m_robotDrive.getHeading();
  }

  public boolean isNavXConnected() {
    return m_robotDrive.isGyroConnected();
  }

  public double getShooterAverageRpm() {
    return m_shooterSubsystem.getAverageRpm();
  }

  public boolean hasLimelightTarget() {
    return m_limelightSubsystem.hasTarget();
  }

  public boolean isLimelightAimingActive() {
    return m_driverController.getLeftBumperButton() && m_limelightSubsystem.hasTarget();
  }

  public void stopShooterAutomation() {
    if (m_autoShootLoopCommand != null) {
      m_autoShootLoopCommand.cancel();
      m_autoShootLoopCommand = null;
    }
    m_shooterSubsystem.stop();
    m_conveyorSubsystem.stop();
    m_feederSubsystem.stop();
    m_shootingActive = false;
  }
}

