// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static final class DriveConstants {
    public static final int kLeftMotorPort = 0;
    public static final int kRightMotorPort = 0;

    public static final double kDriveTick2Feet = 1.0 / 128 * 6 * Math.PI / 12;

    // values are related to PID and need to be tuned:
    public static final double kP = 0.5;
    public static final double kI = 0.5;
    public static final double kD = 0.1;

  }

  public static final class IntakeConstants {

    public static int launchWheelPort;

  }

  public static final class ShooterConstants {

    public static double kIntakeLauncherSpeed;
    public static double kIntakeFeederSpeed;
    public static int launchWheelPort;
    public static int intakeWheelPort;

  }

  public static final class OIConstants {
    public static final int kDriverJoystickPort = 0;

    public static final int kArcadeDriveSpeedAxis = 1;
    public static final int kArcadeDriveTurnAxis = 3;
  }
}