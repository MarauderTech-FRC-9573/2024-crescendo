package frc.robot;

import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.LaunchAmp;
import frc.robot.commands.LaunchSpeaker;
import frc.robot.commands.PrepareLaunchAmp;
import frc.robot.commands.PrepareLaunchSpeaker;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.commands.TurnToAngle;
import frc.robot.commands.TurnToAngleProfiled;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;

public class RobotContainer {
  PowerDistribution powerDistribution = new PowerDistribution();
  
  private final DriveSubsystem driveSubsystem = new DriveSubsystem();
  private final IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
  private final ShooterSubsystem shooterSubsystem = new ShooterSubsystem();
  private final VisionSubsystem visionSubsystem = new VisionSubsystem();
  
  private final CommandXboxController driveController = new CommandXboxController(DriveConstants.driveControllerPort);
  private final CommandXboxController operatorController = new CommandXboxController(DriveConstants.operatorControllerPort);
  
  public RobotContainer() {
    configureButtonBindings();
    driveSubsystem.setDefaultCommand(new RunCommand(() -> driveSubsystem.driveArcade(-driveController.getLeftY(), -driveController.getRightX()), driveSubsystem));
    // Raspberry Pi Vision
    powerDistribution.setSwitchableChannel(true);
    
  }
  
  private void configureButtonBindings() {
    /*Create an inline sequence to run when the operator presses and holds the A (green) button. Run the PrepareLaunch
    * command for 1 seconds and then run the LaunchNote command */
    operatorController.a().whileTrue(new PrepareLaunchSpeaker(shooterSubsystem).withTimeout(ShooterConstants.kLauncherDelay).andThen(new LaunchSpeaker(shooterSubsystem)).handleInterrupt(() -> shooterSubsystem.stop()));
    operatorController.b().whileTrue(new PrepareLaunchAmp(shooterSubsystem).withTimeout(ShooterConstants.kLauncherDelay).andThen(new LaunchAmp(shooterSubsystem)).handleInterrupt(() -> shooterSubsystem.stop()));
    
    // Set up a binding to run the intake command while the operator is pressing and holding the left Bumper
    operatorController.leftBumper().whileTrue(shooterSubsystem.getIntakeCommand());
    
    //New commands from this branch specifically, idk why they were removed
    driveController.x().whileTrue(shooterSubsystem.getIntakeCommand());
    driveController.y().whileTrue(shooterSubsystem.getLaunchCommand());
    
    driveController.rightBumper()
    .whileTrue(new InstantCommand(() -> driveSubsystem.setMaxOutput(0.1)))
    .whileFalse(new InstantCommand(() -> driveSubsystem.setMaxOutput(1.0)));
    
    
    
    
  }
  
  // public Command getAutonomousCommand() {
    //   return Autos.exampleAuto(driveSubsystem, driveController);
    //   }
  }
  
  