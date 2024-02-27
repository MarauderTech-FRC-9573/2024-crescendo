package frc.robot;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.Autos;
import frc.robot.commands.LaunchAmp;
import frc.robot.commands.LaunchSpeaker;
import frc.robot.commands.PrepareLaunchAmp;
import frc.robot.commands.PrepareLaunchSpeaker;
import frc.robot.commands.TurnToAngleProfiled;
import frc.robot.commands.TurnToAngle;
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
    //operatorController.a().whileTrue(new PrepareLaunchSpeaker(shooterSubsystem).withTimeout(1).andThen(new LaunchSpeaker(shooterSubsystem)).handleInterrupt(() -> shooterSubsystem.stop()));
    //operatorController.b().whileTrue(new PrepareLaunchAmp(shooterSubsystem).withTimeout(1).andThen(new LaunchAmp(shooterSubsystem)).handleInterrupt(() -> shooterSubsystem.stop()));

    // Set up a binding to run the intake command while the operator is pressing and holding the left Bumper
    //operatorController.leftBumper().whileTrue(shooterSubsystem.getIntakeCommand());

    // Scales Robot speed
    operatorController.rightBumper()
        .onTrue(new InstantCommand(() -> driveSubsystem.setMaxOutput(0.5)))
        .onFalse(new InstantCommand(() -> driveSubsystem.setMaxOutput(1)));

    // Stabilize robot to drive straight with gyro when left bumper is held
    operatorController.leftBumper()
        .whileTrue(
            new PIDCommand(
                new PIDController(
                    DriveConstants.kStabilizationP,
                    DriveConstants.kStabilizationI,
                    DriveConstants.kStabilizationD),
                // Close the loop on the turn rate
                driveSubsystem::getTurnRate,
                // Setpoint is 0
                0,
                // Pipe the output to the turning controls
                output -> driveSubsystem.arcadeDrive(-operatorController.getLeftY(), output),
                // Require the robot drive
                driveSubsystem));

    // Turn to 90 degrees when the 'X' button is pressed, with a 5 second timeout
    operatorController.x()
        .onTrue(new TurnToAngle(90, driveSubsystem).withTimeout(5));

    // Turn to -90 degrees with a profile when the Circle button is pressed, with a 5 second timeout
    operatorController.y()
        .onTrue(new TurnToAngleProfiled(-90, driveSubsystem).withTimeout(5));
    
  }
  
  public Command getAutonomousCommand() {
    return Autos.exampleAuto(driveSubsystem, driveController);
    }
  }
  
  