// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.commands.LaunchSpeaker;
import frc.robot.commands.PrepareLaunchSpeaker;
import frc.robot.commands.DriveForwardCmd;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import frc.robot.commands.TurnToAngle;
import frc.robot.commands.TurnToAngleProfiled;
import frc.robot.commands.LaunchAmp;
import frc.robot.commands.PrepareLaunchAmp;
import edu.wpi.first.wpilibj2.command.InstantCommand;


/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */

public class RobotContainer {
  // The robot's subsystems
  private final DriveSubsystem m_drive = new DriveSubsystem();
  private final ShooterSubsystem m_shooter = new ShooterSubsystem();
  // The driver's controller
  private final CommandXboxController m_driverController = new CommandXboxController(OIConstants.driveControllerPort);

  private final SendableChooser<Command> m_autoChooser = new SendableChooser<Command>();

  public RobotContainer() {

    initializeSubsystems();
    configureBindings();
    initializeAutoChooser();
    SmartDashboard.putData(m_autoChooser);

  }

  public void initializeSubsystems() {

    // Control the drive with split-stick arcade controls
    m_drive.setDefaultCommand(
        m_drive.arcadeSysId(
            () -> m_driverController.getLeftY(), () -> -m_driverController.getRightX()));

    SmartDashboard.putData(m_drive);
    SmartDashboard.putData(m_shooter);
    

  }

  public void initializeAutoChooser() {

    m_autoChooser.setDefaultOption("Drive forward: ", 
        new WaitCommand(0.1)
            .andThen(new DriveForwardCmd(m_drive, 10, 0.5))
            .withTimeout(3)
            .andThen(new RunCommand(() -> m_drive.driveArcade(0,0), m_drive)));
    
    m_autoChooser.addOption("SysID Quasistatic Foward(backward): ", 
        m_drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    
    m_autoChooser.addOption("SysID Quasistatic Backward(real forward): ", 
        m_drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
        
    m_autoChooser.addOption("SysID Dynamic Foward(backward): ", 
        m_drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
    
    m_autoChooser.addOption("SysID Dynamic Backward(real forward): ", 
        m_drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));



  }

  /**
   * Use this method to define bindings between conditions and commands. These are useful for
   * automating robot behaviors based on button and sensor input.
   *
   * <p>Should be called during {@link Robot#robotInit()}.
   *
   * <p>Event binding methods are available on the {@link Trigger} class.
   */
  public void configureBindings() {

    m_driverController
        .a().whileTrue(new PrepareLaunchSpeaker(m_shooter).withTimeout(ShooterConstants.kLauncherDelay).andThen(new LaunchSpeaker(m_shooter)).handleInterrupt(() -> m_shooter.stop()));
    
    m_driverController
        .b().whileTrue(new PrepareLaunchAmp(m_shooter).withTimeout(ShooterConstants.kLauncherDelay).andThen(new LaunchAmp(m_shooter)).handleInterrupt(() -> m_shooter.stop()));
    
    m_driverController.leftBumper().whileTrue(m_shooter.getIntakeCommand());
    
    m_driverController.rightBumper()
        .whileTrue(new InstantCommand(() -> m_drive.setMaxOutput(0.1)))
        .whileFalse(new InstantCommand(() -> m_drive.setMaxOutput(1.0)));

    m_driverController
        .a()
        .whileTrue(new WaitCommand(0.1)
        .andThen(new TurnToAngle(90, m_drive).withTimeout(1)));

    // Turn to -90 degrees with a profile when the Circle button is pressed, with a 5 second timeout
    m_driverController
        .b()
        .whileTrue(new WaitCommand(0.1)
        .andThen(new TurnToAngleProfiled(-90, m_drive).withTimeout(1)));

  }

  /**
   * Use this to define the command that runs during autonomous.
   *
   * <p>Scheduled during {@link Robot#autonomousInit()}.
   */
  public Command getAutonomousCommand() {
    // Do nothing
    return m_autoChooser.getSelected();
  }
}