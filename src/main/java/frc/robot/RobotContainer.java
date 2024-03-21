package frc.robot;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.commands.IntakeArmSwitch;
import frc.robot.commands.LaunchAmp;
import frc.robot.commands.LaunchSpeaker;
import frc.robot.commands.PrepareLaunchAmp;
import frc.robot.commands.PrepareLaunchSpeaker;
import frc.robot.commands.IntakeArmSwitch;
import frc.robot.commands.IntakeReceiver;
import frc.robot.commands.IntakeReleaser;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.commands.Autos;
import frc.robot.commands.DriveForwardCmd;

public class RobotContainer {
  private final DriveSubsystem driveSubsystem = new DriveSubsystem();
  private final IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
  private final ShooterSubsystem shooterSubsystem = new ShooterSubsystem();
  private final VisionSubsystem visionSubsystem = new VisionSubsystem();
  
  private final CommandXboxController driveController = new CommandXboxController(DriveConstants.driveControllerPort);
  private final CommandXboxController operatorController = new CommandXboxController(DriveConstants.operatorControllerPort);
  
  private final PowerDistribution pdh = new PowerDistribution(1, ModuleType.kRev);
  
  private final SendableChooser<Command> m_autoChooser = new SendableChooser<Command>();
  
  
  public RobotContainer() {
    pdh.setSwitchableChannel(true);
    configureButtonBindings();
    SmartDashboard.putData(m_autoChooser);
  }
  
  public void initializeSubsystems() { 
    driveSubsystem.setDefaultCommand(new RunCommand(() -> driveSubsystem.arcadeDrive(-driveController.getLeftY(), -driveController.getRightX()), driveSubsystem));
    intakeSubsystem.setDefaultCommand(new RunCommand(() -> intakeSubsystem.runArm(() -> driveController.getLeftY())));
    SmartDashboard.putData(intakeSubsystem);
  }
  
  public void initializeAutoChooser() {
    m_autoChooser.setDefaultOption("Drive forward: ",
    new WaitCommand(0.1).andThen(new DriveForwardCmd(driveSubsystem, 10, 0.5)).withTimeout(1).andThen(new RunCommand(() -> driveSubsystem.arcadeDrive(0, 0), driveSubsystem)));
    
    m_autoChooser.addOption("SysID Quasistatic Forward: ",
    intakeSubsystem.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    
    m_autoChooser.addOption("SysId Quasistatic Backward: ", intakeSubsystem.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    
    m_autoChooser.addOption("SysID Dynamic Forward: ", intakeSubsystem.sysIdDynamic(SysIdRoutine.Direction.kForward));
    
    m_autoChooser.addOption("SysID Dynamic Backward: ", intakeSubsystem.sysIdDynamic(SysIdRoutine.Direction.kReverse));
    
  }
  
  private void configureButtonBindings() {
    /*Create an inline sequence to run when the operator presses and holds the A (green) button. Run the PrepareLaunch
    * command for 1 seconds and then run the LaunchNote command */
    // operatorController.a().whileTrue(new PrepareLaunchSpeaker(shooterSubsystem).withTimeout(1).andThen(new LaunchSpeaker(shooterSubsystem)).handleInterrupt(() -> shooterSubsystem.stop()));
    // operatorController.b().whileTrue(new PrepareLaunchAmp(shooterSubsystem).withTimeout(1).andThen(new LaunchAmp(shooterSubsystem)).handleInterrupt(() -> shooterSubsystem.stop()));
    
    // Set up a binding to run the intake command while the operator is pressing and holding the left Bumper
    operatorController.leftBumper().whileTrue(shooterSubsystem.getIntakeCommand());
    
    // Ground Intake Button Bindings
    operatorController.rightBumper().whileTrue(new IntakeArmSwitch(intakeSubsystem).handleInterrupt(() -> intakeSubsystem.stop()));
    // operatorController.x().whileTrue(new IntakeReceiver(intakeSubsystem).handleInterrupt(() -> intakeSubsystem.stop()));
    // operatorController.y().whileTrue(new IntakeReleaser(intakeSubsystem).withTimeout(1).handleInterrupt(() -> intakeSubsystem.stop()));
    
    operatorController.x().whileTrue(intakeSubsystem.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    operatorController.y().whileTrue(intakeSubsystem.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    operatorController.a().whileTrue(intakeSubsystem.sysIdDynamic(SysIdRoutine.Direction.kForward));
    operatorController.b().whileTrue(intakeSubsystem.sysIdDynamic(SysIdRoutine.Direction.kReverse));
    
    
  }
  
  public Command getAutonomousCommand() {
    return m_autoChooser.getSelected();
  }
}

