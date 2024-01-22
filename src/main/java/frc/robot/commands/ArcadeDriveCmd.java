// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Supplier;
import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

/** An example command that uses an example subsystem. */
public class ArcadeDriveCmd extends Command {
  private final double DEADBAND = 0.05;
  private final DriveSubsystem driveSubsystem;
  private final CommandXboxController controller;

  // initializer for the ArcadeDriveCmd, taking in a driveSubsystem instance, and speed & turn 
  public ArcadeDriveCmd(DriveSubsystem driveSubsystem, CommandXboxController controller) {
    this.driveSubsystem = driveSubsystem;
    this.controller = controller;
    addRequirements(driveSubsystem);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    double xSpeed = -controller.getLeftY();
    double ySpeed = -controller.getLeftX();
    
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
