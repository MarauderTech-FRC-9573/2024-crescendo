package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.AimAtTarget;
import frc.robot.commands.Autos;
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

public class RobotContainer {
  private final DriveSubsystem driveSubsystem = new DriveSubsystem();
  private final IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
  private final ShooterSubsystem shooterSubsystem = new ShooterSubsystem();
  private final VisionSubsystem visionSubsystem = new VisionSubsystem();

  private final CommandXboxController driveController = new CommandXboxController(DriveConstants.driveControllerPort);
  private final CommandXboxController operatorController = new CommandXboxController(DriveConstants.operatorControllerPort);
  
  public RobotContainer() {
    configureButtonBindings();
    driveSubsystem.setDefaultCommand(new RunCommand(() -> driveSubsystem.arcadeDrive(-driveController.getLeftY(), -driveController.getRightX()), driveSubsystem));
    
  }
  
  private void configureButtonBindings() {
    /*Create an inline sequence to run when the operator presses and holds the A (green) button. Run the PrepareLaunch
     * command for 1 seconds and then run the LaunchNote command */
    operatorController.a().whileTrue(new PrepareLaunchSpeaker(shooterSubsystem).withTimeout(1).andThen(new LaunchSpeaker(shooterSubsystem)).handleInterrupt(() -> shooterSubsystem.stop()));
    operatorController.b().whileTrue(new PrepareLaunchAmp(shooterSubsystem).withTimeout(1).andThen(new LaunchAmp(shooterSubsystem)).handleInterrupt(() -> shooterSubsystem.stop()));

    // Set up a binding to run the intake command while the operator is pressing and holding the left Bumper
    operatorController.leftBumper().whileTrue(shooterSubsystem.getIntakeCommand());
    operatorController.x().whileTrue(new AimAtTarget(visionSubsystem, driveSubsystem, operatorController).withTimeout(1));
    
  }
  
  // public Command getAutonomousCommand() {
  //   return new AimAtTarget(visionSubsystem);
  // }
}
  
  