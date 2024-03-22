package frc.robot;

import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.Constants.DriveConstants;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;

public class RobotContainer {
  private final DriveSubsystem driveSubsystem = new DriveSubsystem();
  private final IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
  private final ShooterSubsystem shooterSubsystem = new ShooterSubsystem();

  private final CommandXboxController driveController = new CommandXboxController(DriveConstants.driveControllerPort);
  private final CommandXboxController operatorController = new CommandXboxController(DriveConstants.operatorControllerPort);

  private final PowerDistribution pdh = new PowerDistribution(1, ModuleType.kRev);

  private final SendableChooser<Command> m_autoChooser = new SendableChooser<Command>();

  
  public RobotContainer() {
    initalizeAutoChooser();
    pdh.setSwitchableChannel(true);
    configureButtonBindings();
    driveSubsystem.setDefaultCommand(new RunCommand(() -> driveSubsystem.driveArcade(-driveController.getLeftY(), -driveController.getRightX()), driveSubsystem));
    
  }
  
  private void configureButtonBindings() {
    /*Create an inline sequence to run when the operator presses and holds the A (green) button. Run the PrepareLaunch
     * command for 1 seconds and then run the LaunchNote command */
    driveController.a().whileTrue(new LaunchSpeaker(shooterSubsystem));
    driveController.b().whileTrue(new LaunchAmp(shooterSubsystem));

    // Set up a binding to run the intake command while the operator is pressing and holding the left Bumper
    driveController.leftBumper().whileTrue(new IntakeSource(shooterSubsystem));

    //New commands from this branch specifically, idk why they were removed

    driveController.rightBumper()
        .whileTrue(new InstantCommand(() -> driveSubsystem.setMaxOutput(0.1)))
        .whileFalse(new InstantCommand(() -> driveSubsystem.setMaxOutput(1.0)));

    
    // Arm Ground Intake Button Bindings
    operatorController.leftBumper().whileTrue(new ArmForward(intakeSubsystem).handleInterrupt(() -> intakeSubsystem.stop()));
    operatorController.rightBumper().whileTrue(new ArmBackward(intakeSubsystem).handleInterrupt(() -> intakeSubsystem.stop()));
    
    // Intake Ground Intake Button Bindings
    operatorController.x().whileTrue(new IntakeReceiver(intakeSubsystem).handleInterrupt(() -> intakeSubsystem.stop()));
    operatorController.y().whileTrue(new IntakeReleaser(intakeSubsystem, shooterSubsystem).withTimeout(1).handleInterrupt(() -> intakeSubsystem.stop()));
  
  }

  public void initalizeAutoChooser() {
      
      m_autoChooser.setDefaultOption("Drive forward: ", 
      new WaitCommand(0.1)
      .andThen(new RunCommand(() -> driveSubsystem.driveArcade(0.5, 0), driveSubsystem))
      .withTimeout(3)
      .andThen(new RunCommand(() -> driveSubsystem.driveArcade(0, 0), driveSubsystem)));

  }
  
  public Command getAutonomousCommand() {
      return m_autoChooser.getSelected();
    
    }
}
  
  