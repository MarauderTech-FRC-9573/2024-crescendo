package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.ArcadeDriveCmd;
import frc.robot.commands.DriveForwardCmd;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.Constants.OIConstants;

public class RobotContainer {
  private final DriveSubsystem driveSubsystem = new DriveSubsystem();
  private final Joystick joystick1 = new Joystick(0); // Fix: Added joystick1 declaration
  
  public RobotContainer() {
    configureButtonBindings();
    
    driveSubsystem.setDefaultCommand(new ArcadeDriveCmd(driveSubsystem, () -> -joystick1.getRawAxis(OIConstants.kArcadeDriveSpeedAxis), () -> joystick1.getRawAxis(OIConstants.kArcadeDriveTurnAxis)));
  }
  
  private void configureButtonBindings() {
    new JoystickButton(joystick1, 0).whileTrue(new DriveForwardCmd(driveSubsystem, 10));
  }
  
  // public Command getAutonomousCommand() {
    
    // }
  }
  
