package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.LaunchAmp;
import frc.robot.commands.LaunchSpeaker;
import frc.robot.subsystems.ClimberSubsystem;
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
import frc.robot.commands.ClimbStage;
// import frc.robot.commands.IntakeSource;

public class RobotContainer {
  private final DriveSubsystem driveSubsystem = new DriveSubsystem();
  private final IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
  private final ShooterSubsystem shooterSubsystem = new ShooterSubsystem();
  private final VisionSubsystem visionSubsystem = new VisionSubsystem();
  private final ClimberSubsystem  climberSubsystem = new ClimberSubsystem();

  private final CommandXboxController driveController = new CommandXboxController(0);
  private final CommandXboxController operatorController = new CommandXboxController(1);
  
  public RobotContainer() {
    configureButtonBindings();
    driveSubsystem.setDefaultCommand(new RunCommand(() -> driveSubsystem.driveArcade(-driveController.getLeftY(), -driveController.getRightX()), driveSubsystem));
    
  }
  
  private void configureButtonBindings() {
    /*Create an inline sequence to run when the operator presses and holds the A (green) button. Run the PrepareLaunch
     * command for 1 seconds and then run the LaunchNote command */
    driveController.a().whileTrue(new LaunchSpeaker(shooterSubsystem));
    driveController.b().whileTrue(new LaunchAmp(shooterSubsystem));
    driveController.x().whileTrue(new ClimbStage(climberSubsystem, true));

    // Set up a binding to run the intake command while the operator is pressing and holding the left Bumper
    // driveController.leftBumper().whileTrue(new IntakeSource(shooterSubsystem));

    //New commands from this branch specifically, idk why they were removed

    driveController.rightBumper()
        .whileTrue(new InstantCommand(() -> driveSubsystem.setMaxOutput(0.1)))
        .whileFalse(new InstantCommand(() -> driveSubsystem.setMaxOutput(1.0)));

    
  }
  
  // public Command getAutonomousCommand() {
  //   return Autos.exampleAuto(driveSubsystem, driveController);
  //   }
  }
  
  