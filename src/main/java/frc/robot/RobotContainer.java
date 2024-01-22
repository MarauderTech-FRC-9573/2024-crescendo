package frc.robot;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
\import frc.robot.commands.ArcadeDriveCmd;
// import frc.robot.commands.DriveForwardCmd;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;

public class RobotContainer {
  private final DriveSubsystem driveSubsystem = new DriveSubsystem();
  private final IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
  private final ShooterSubsystem shooterSubsystem = new ShooterSubsystem();
  private final CommandXboxController joystick1 = new CommandXboxController(DriveConstants.joystickPort);
  
  public RobotContainer() {
    configureButtonBindings();

    driveSubsystem.setDefaultCommand(new ArcadeDriveCmd(driveSubsystem, joystick1));
  }
  
  private void configureButtonBindings() {
    // new JoystickButton(joystick1, 1).whileTrue(new DriveForwardCmd(driveSubsystem, 10));
  }
  
  // public Command getAutonomousCommand() {
    
    // }
  }
  
