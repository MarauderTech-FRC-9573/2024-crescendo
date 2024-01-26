package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.DriveForwardCmd;
import frc.robot.commands.LaunchNote;
import frc.robot.commands.PrepareLaunch;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.ShooterConstants;

public class RobotContainer {
  private final DriveSubsystem driveSubsystem = new DriveSubsystem();
  private final IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
  private final ShooterSubsystem shooterSubsystem = new ShooterSubsystem();
  
  private final CommandXboxController driveController = new CommandXboxController(DriveConstants.driveControllerPort);
  private final CommandXboxController operatorController = new CommandXboxController(DriveConstants.operatorControllerPort);
  
  public RobotContainer() {
    configureButtonBindings();
    driveSubsystem.setDefaultCommand(new RunCommand(() -> driveSubsystem.arcadeDrive(-driveController.getLeftY(), -driveController.getRightX(), driveController), driveSubsystem));
    
  }
  
  private void configureButtonBindings() {
    operatorController.a().whileTrue(new PrepareLaunch(shooterSubsystem).withTimeout(ShooterConstants.kLauncherDelay).andThen(new LaunchNote(shooterSubsystem)).handleInterrupt(() -> shooterSubsystem.stop()));
    
    operatorController.leftBumper().whileTrue(shooterSubsystem.getIntake());
    
  }
  
  // public Command getAutonomousCommand() {
    
    // }
  }
  
  