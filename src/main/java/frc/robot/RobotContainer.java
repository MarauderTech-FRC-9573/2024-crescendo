package frc.robot;

import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.LaunchAmp;
import frc.robot.commands.LaunchSpeaker;
import frc.robot.commands.PrepareLaunchAmp;
import frc.robot.commands.PrepareLaunchSpeaker;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ShooterConstants;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;

public class RobotContainer {
  private final DriveSubsystem driveSubsystem = new DriveSubsystem();
  private final ShooterSubsystem shooterSubsystem = new ShooterSubsystem();
  private final VisionSubsystem  visionSubsystem = new VisionSubsystem();
  private final CommandXboxController operatorController = new CommandXboxController(DriveConstants.operatorControllerPort);

  //private final PowerDistribution pdh = new PowerDistribution(1, ModuleType.kRev);

  private final SendableChooser<Command> m_autoChooser = new SendableChooser<Command>();

  
  public RobotContainer() {
    initalizeAutoChooser();
    //pdh.setSwitchableChannel(true);
    configureButtonBindings();
    driveSubsystem.setDefaultCommand(new RunCommand(() -> driveSubsystem.driveArcade(-operatorController.getLeftY(), -operatorController.getRightX()), driveSubsystem));
    SmartDashboard.putData("Autos: ", m_autoChooser);

  }
  
  private void configureButtonBindings() {
    operatorController.a().whileTrue(new PrepareLaunchSpeaker(shooterSubsystem).withTimeout(ShooterConstants.kLauncherDelay).andThen(new LaunchSpeaker(shooterSubsystem)).handleInterrupt(() -> shooterSubsystem.stop()));
    operatorController.b().whileTrue(new PrepareLaunchAmp(shooterSubsystem).withTimeout(0.5).andThen(new LaunchAmp(shooterSubsystem)).handleInterrupt(() -> shooterSubsystem.stop()));

    // Set up a binding to run the intake command while the operator is pressing and holding the left Bumper
    operatorController.leftBumper().whileTrue(shooterSubsystem.getIntakeCommand());

    operatorController.rightBumper()
        .whileTrue(new InstantCommand(() -> driveSubsystem.setMaxOutput(0.1)))
        .whileFalse(new InstantCommand(() -> driveSubsystem.setMaxOutput(DriveConstants.maxSpeed)));
  
  }

  public void initalizeAutoChooser() {
      
      m_autoChooser.setDefaultOption("Drive forward: ", 
      new WaitCommand(0.1)
      .andThen(new RunCommand(() -> driveSubsystem.driveArcade(0.5, 0), driveSubsystem))
      .withTimeout(3)
      .andThen(new RunCommand(() -> driveSubsystem.driveArcade(0, 0), driveSubsystem)));
      m_autoChooser.addOption("Speaker note: ", new WaitCommand(0.1).andThen(new RunCommand(() -> driveSubsystem.driveArcade(0.5, 0), driveSubsystem)).withTimeout(3).andThen(new RunCommand(() -> LaunchSpeaker(shooterSubsystem), shooterSubsystem)));
  }
  
  private Object LaunchSpeaker(ShooterSubsystem shooterSubsystem2) {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'LaunchSpeaker'");
  }

  public Command getAutonomousCommand() {
      return m_autoChooser.getSelected();
    
    }
}
  
  