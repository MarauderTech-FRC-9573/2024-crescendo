package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;

public class RobotContainer {
  private final DriveSubsystem driveSubsystem = new DriveSubsystem();

  public RobotContainer() {
    configureButtonBindings();
  }

  private void configureButtonBindings() {

  }

  // public Command getAutonomousCommand() {
    
  // }
}