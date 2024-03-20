package frc.robot;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.ArmMoveBackward;
import frc.robot.commands.ArmMoveForward;
import frc.robot.commands.LaunchAmp;
import frc.robot.commands.LaunchSpeaker;
import frc.robot.commands.PrepareLaunchAmp;
import frc.robot.commands.PrepareLaunchSpeaker;
import frc.robot.commands.ArmMoveForward;
import frc.robot.commands.IntakeReceiver;
import frc.robot.commands.IntakeReleaser;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.commands.Autos;

public class RobotContainer {
  private final DriveSubsystem driveSubsystem = new DriveSubsystem();
  private final IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
  private final ShooterSubsystem shooterSubsystem = new ShooterSubsystem();
  private final VisionSubsystem visionSubsystem = new VisionSubsystem();

  private final CommandXboxController driveController = new CommandXboxController(DriveConstants.driveControllerPort);
  private final CommandXboxController operatorController = new CommandXboxController(DriveConstants.operatorControllerPort);

  private final PowerDistribution pdh = new PowerDistribution(1, ModuleType.kRev);
  
  public RobotContainer() {
    pdh.setSwitchableChannel(true);
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
    
    // Ground Intake Button Bindings
    operatorController.x().whileTrue(new ArmMoveForward(intakeSubsystem).handleInterrupt(() -> intakeSubsystem.stop()));
    operatorController.y().whileTrue(new ArmMoveBackward(intakeSubsystem).handleInterrupt(() -> intakeSubsystem.stop()));
    operatorController.rightBumper().whileTrue(new IntakeReceiver(intakeSubsystem).handleInterrupt(() -> intakeSubsystem.stop()));
  
  }
  
  public Command getAutonomousCommand() {
    return Autos.exampleAuto(driveSubsystem, driveController);
    }
  }
  
  