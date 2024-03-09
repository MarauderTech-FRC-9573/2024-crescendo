package frc.robot;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.LaunchAmp;
import frc.robot.commands.LaunchSpeaker;
import frc.robot.commands.PrepareLaunchAmp;
import frc.robot.commands.PrepareLaunchSpeaker;
import frc.robot.commands.TurnToAngleProfiled;
import frc.robot.commands.autonomous.AimAndRange;
import frc.robot.commands.TurnToAngle;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ShooterConstants;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;

public class RobotContainer {
  
  public final SendableChooser<Command> m_autoChooser = new SendableChooser<Command>();
  
  PowerDistribution pdp = new PowerDistribution();

  private final DriveSubsystem driveSubsystem = new DriveSubsystem();
  private final IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
  private final ShooterSubsystem shooterSubsystem = new ShooterSubsystem();
  private final VisionSubsystem visionSubsystem = new VisionSubsystem();
  
  private final CommandXboxController driveController = new CommandXboxController(DriveConstants.driveControllerPort);
  private final CommandXboxController operatorController = new CommandXboxController(DriveConstants.operatorControllerPort);
  
  public RobotContainer() {
    configureButtonBindings();
    
    driveSubsystem.setDefaultCommand(new RunCommand(() -> driveSubsystem.driveArcade(-driveController.getLeftY(), -driveController.getRightX()), driveSubsystem));
    
    visionSubsystem.setDefaultCommand(new RunCommand(() -> visionSubsystem.getAprilTags(), visionSubsystem));

    pdp.setSwitchableChannel(true);
    
  }

  public void initializeAutoChooser(){
    m_autoChooser.setDefaultOption(
      "Aim and Range",
      new AimAndRange(visionSubsystem, driveSubsystem).withTimeout(15));
    m_autoChooser.addOption(
      "Turn to 90",
      new TurnToAngle(90, driveSubsystem));
  }
  
  private void configureButtonBindings() {
    /*Create an inline sequence to run when the operator presses and holds the A (green) button. Run the PrepareLaunch
    * command for 1 seconds and then run the LaunchNote command */
    //driveController.x().whileTrue(new PrepareLaunchSpeaker(shooterSubsystem).withTimeout(1).andThen(new LaunchSpeaker(shooterSubsystem)).handleInterrupt(() -> shooterSubsystem.stop()));
    //driveController.y().whileTrue(new PrepareLaunchAmp(shooterSubsystem).withTimeout(1).andThen(new LaunchAmp(shooterSubsystem)).handleInterrupt(() -> shooterSubsystem.stop()));
    
    // Set up a binding to run the intake command while the operator is pressing and holding the left Bumper
    //operatorController.leftBumper().whileTrue(shooterSubsystem.getIntakeCommand());
    
    // Scales Robot speed
    driveController.rightBumper()
    .whileTrue(new InstantCommand(() -> driveSubsystem.setMaxOutput(0.1)))
    .whileFalse(new InstantCommand(() -> driveSubsystem.setMaxOutput(1.0)));
    // Stabilize robot to drive straight with gyro when left bumper is held
    driveController.leftBumper()
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
    output -> driveSubsystem.driveArcade(-driveController.getLeftY(), output),
    // Require the robot drive
    driveSubsystem)); 
    
    // Turn to 90 degrees when the 'X' button is pressed, with a 5 second timeout
    driveController.a()
    .whileTrue(new WaitCommand(0.1).andThen(new TurnToAngle(90, driveSubsystem).withTimeout(1)));
    
    // Turn to -90 degrees with a profile when the Circle button is pressed, with a 5 second timeout
    driveController.b()
    .whileTrue(new WaitCommand(0.1).andThen(new TurnToAngleProfiled(-90, driveSubsystem).withTimeout(1)));
    
    driveController.leftBumper().whileTrue(shooterSubsystem.getIntakeCommand());
    
    driveController.x().whileTrue(new AimAndRange(visionSubsystem, driveSubsystem).withTimeout(1));
    
  }
  
  public Command getAutonomousCommand() {
    return m_autoChooser.getSelected(); 
  }
}
