// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.DriveSubsystem;

// import frc.robot.subsystems.CANDrivetrain;

public final class Autos {
  /** Example static factory for an autonomous command. */
public static Command exampleAuto(DriveSubsystem drivetrain, CommandXboxController controller) {
    /**
     * RunCommand is a helper class that creates a command from a single method, in this case we
     * pass it the arcadeDrive method to drive straight back at half power. We modify that command
     * with the .withTimeout(1) decorator to timeout after 1 second, and use the .andThen decorator
     * to stop the drivetrain after the first command times out
     */
    return new RunCommand(() -> drivetrain.arcadeDrive(-.5, 0))
            .withTimeout(1)
            .andThen(new RunCommand(() -> drivetrain.arcadeDrive(0, 0)));
}

        private Autos() {
            throw new UnsupportedOperationException("This is a utility class!");
  }
}
