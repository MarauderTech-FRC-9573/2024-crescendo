package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.ArcadeDriveCmd;
import frc.robot.subsystems.DriveSubsystem;

import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;


public class RobotContainer {
  private final DriveSubsystem driveSubsystem = new DriveSubsystem();
  private final Joystick joystick1 = new Joystick(OIConstants.kDriverJoystickPort);

    public RobotContainer() {
      configureButtonBindings();

      driveSubsystem.setDefaultCommand(new ArcadeDriveCmd(driveSubsystem, () -> -joystick1.getRawAxis(OIConstants.kArcadeDriveSpeedAxis), () -> joystick1.getRawAxis(OIConstants.kArcadeDriveTurnAxis))); // Fix: Added closing parenthesis
    }

    private void configureButtonBindings() {

    }

    // public Command getAutonomousCommand() {
      
    // }
  }
