// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Supplier;
import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.wpilibj2.command.Command;

/** An example command that uses an example subsystem. */
public class ArcadeDriveCmd extends Command {
  private final DriveSubsystem driveSubsystem;
  private final Supplier<Double> speedFunction, turnFunction;

  // initializer for the ArcadeDriveCmd, taking in a driveSubsystem instance, and speed & turn 
  public ArcadeDriveCmd(DriveSubsystem driveSubsystem, Supplier<Double> speedFunction, Supplier<Double> turnFunction) {
    this.driveSubsystem = driveSubsystem;
    this.speedFunction = speedFunction;
    this.turnFunction = turnFunction;
    addRequirements(driveSubsystem);
  }

  @Override
  public void initialize() {
    System.out.println("ArcadeDrive started!");
  }

  @Override
  public void execute() {
    double speed = speedFunction.get();
    double turn = turnFunction.get();

    driveSubsystem.setMaxOutput(speed);
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
