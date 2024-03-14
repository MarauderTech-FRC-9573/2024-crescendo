// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class SysIdRoutineBot {
  // The robot's subsystems
  private final DriveSubsystem m_drive = new DriveSubsystem();
  private final ShooterSubsystem m_shooter = new ShooterSubsystem();

  // The driver's controller
  CommandXboxController m_driverController =
      new CommandXboxController(DriveConstants.driveControllerPort);

  /**
   * Use this method to define bindings between conditions and commands. These are useful for
   * automating robot behaviors based on button and sensor input.
   *
   * <p>Should be called during {@link Robot#robotInit()}.
   *
   * <p>Event binding methods are available on the {@link Trigger} class.
   */
  public void configureBindings() {
    // Control the drive with split-stick arcade controls
    m_driverController
        .a()
        .and(m_driverController.rightBumper())
        .whileTrue(new RunCommand(() -> m_drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward), m_drive));
    m_driverController
        .b()
        .and(m_driverController.rightBumper())
        .whileTrue(new RunCommand(() -> m_drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse), m_drive));
    m_driverController
        .x()
        .and(m_driverController.rightBumper())
        .whileTrue(new RunCommand(() -> m_drive.sysIdDynamic(SysIdRoutine.Direction.kForward), m_drive));
    m_driverController
        .y()
        .and(m_driverController.rightBumper())
        .whileTrue(new RunCommand(() -> m_drive.sysIdDynamic(SysIdRoutine.Direction.kReverse), m_drive));
    m_driverController
        .y()
        .and(m_driverController.rightBumper())
        .whileTrue(m_drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));
  }

  /**
   * Use this to define the command that runs during autonomous.
   *
   * <p>Scheduled during {@link Robot#autonomousInit()}.
   */
  public Command getAutonomousCommand() {
    // Do nothing
    return m_drive.run(() -> {});
  }
}