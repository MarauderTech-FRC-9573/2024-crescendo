// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;

/** An example command that uses an example subsystem. */
public class ArcadeDriveCmd extends Command {
  private final DriveSubsystem driveSubsystem;
  private final XboxController controller;
  
  // initializer for the ArcadeDriveCmd, taking in a driveSubsystem instance, and speed & turn 
  public ArcadeDriveCmd(DriveSubsystem driveSubsystem, XboxController controller) {
    this.driveSubsystem = driveSubsystem;
    this.controller = controller;
    addRequirements(driveSubsystem);
  }
  
  @Override
  public void initialize() {}
  
  @Override
  public void execute() {        
    // Clamping the joystick values to ensure they are within [-1, 1]
    double xSpeed = MathUtil.clamp(-controller.getLeftY(), -1.0, 1.0); // Forward/Backward speed
    double ySpeed = MathUtil.clamp(controller.getLeftX(), -1.0, 1.0);  // Turning speed
    
    // Debugging
    System.out.println("xSpeed: " + xSpeed + ", ySpeed: " + ySpeed);
    
    driveSubsystem.drive(xSpeed, ySpeed);
    
  }
  
  @Override 
  public void end(boolean interrputed)  {
    System.out.println("ArcadeDrive done!");
  }
  
  @Override 
  public boolean isFinished() {
    return false;
  }
}
