// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.ShooterConstants;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.DriveSubsystem;
  import frc.robot.subsystems.IntakeSubsystem;
  import frc.robot.subsystems.ShooterSubsystem;
  import frc.robot.Constants.DriveConstants;
  import edu.wpi.first.wpilibj2.command.InstantCommand;

/**
* This class is where the bulk of the robot should be declared. Since Command-based is a
* "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
* periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
* subsystems, commands, and button mappings) should be declared here.
**/

/**
  * Use this to define the command that runs during autonomous.
  *
  * <p>Scheduled during {@link Robot#autonomousInit()}.
  */
import frc.robot.commands.*;
  
  public class RobotContainer {
    private final DriveSubsystem driveSubsystem = new DriveSubsystem();
    private final IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
    private final ShooterSubsystem shooterSubsystem = new ShooterSubsystem();
    
    private final CommandXboxController driveController = new CommandXboxController(DriveConstants.driveControllerPort);
    private final CommandXboxController operatorController = new CommandXboxController(DriveConstants.operatorControllerPort);
    
    private final PowerDistribution pdh = new PowerDistribution(1, ModuleType.kRev);

      private final SendableChooser<Command> m_autoChooser = new SendableChooser<Command>();

    
    public RobotContainer() {
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
      operatorController.y().whileTrue(new IntakeReleaser(intakeSubsystem).withTimeout(1).handleInterrupt(() -> intakeSubsystem.stop()));
      
      // FOR SYSID
      //     m_driverController
      //     .a().whileTrue(new PrepareLaunchSpeaker(m_shooter).withTimeout(ShooterConstants.kLauncherDelay).andThen(new LaunchSpeaker(m_shooter)).handleInterrupt(() -> m_shooter.stop()));
      
      // m_driverController
      //     .b().whileTrue(new PrepareLaunchAmp(m_shooter).withTimeout(ShooterConstants.kLauncherDelay).andThen(new LaunchAmp(m_shooter)).handleInterrupt(() -> m_shooter.stop()));
      
      // m_driverController.leftBumper().whileTrue(m_shooter.getIntakeCommand());
      
      // m_driverController.rightBumper()
      //     .whileTrue(new InstantCommand(() -> m_drive.setMaxOutput(0.1)))
      //     .whileFalse(new InstantCommand(() -> m_drive.setMaxOutput(1.0)));
      
      // m_driverController
      //     .a()
      //     .whileTrue(new WaitCommand(0.1)
      //     .andThen(new TurnToAngle(90, m_drive).withTimeout(1)));
      
      // // Turn to -90 degrees with a profile when the Circle button is pressed, with a 5 second timeout
      // m_driverController
      //     .b()
      //     .whileTrue(new WaitCommand(0.1)
      //     .andThen(new TurnToAngleProfiled(-90, m_drive).withTimeout(1)));
      
      
    }
    
    public void initializeAutoChooser() {
      
      m_autoChooser.setDefaultOption("Drive forward: ", 
      new WaitCommand(0.1)
      .andThen(new DriveForwardCmd(driveSubsystem, 10, 0.5))
      .withTimeout(3)
      .andThen(new RunCommand(() -> driveSubsystem.driveArcade(0,0), driveSubsystem)));
      
      m_autoChooser.addOption("SysID Quasistatic Foward(backward): ", 
      driveSubsystem.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
      
      m_autoChooser.addOption("SysID Quasistatic Backward(real forward): ", 
      driveSubsystem.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
      
      m_autoChooser.addOption("SysID Dynamic Foward(backward): ", 
      driveSubsystem.sysIdDynamic(SysIdRoutine.Direction.kForward));
      
      m_autoChooser.addOption("SysID Dynamic Backward(real forward): ", 
      driveSubsystem.sysIdDynamic(SysIdRoutine.Direction.kReverse));
      
      
      
    }
    
    
    
    public Command getAutonomousCommand() {
      return m_autoChooser.getSelected();
    }
    }
    