// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class FieldConstants {
    public static final double kFieldLength = 4.0;
    public static final double kFieldWidth = 3.0;
  }
  
  public static final class DriveConstants {
    public static final int kLeftMotorPort = 0;
    public static final int kRightMotorPort = 0;
  }

  public static final class IntakeConstants {

  }

  public static final class ShooterConstants {

  }

  public static final class VisionConstants {
    public static final String kCameraName1 = "Camera1";
    public static final String kCameraName2 = "Camera2";
    public static final Transform3d kCamera1ToRobotOffset = new Transform3d();
    public static final Transform3d kCamera2ToRobotOffset = new Transform3d(0.0, -0.12, 0.0, new Rotation3d());


    public static final double kCamera1HeightMeters = 0.215;
    public static final double kCamera2HeightMeters = 0.0;
    public static final double kCamera3HeightMeters = 0.0;

    public static final double KCameraPitchRadians = 0.0;

    public static final double kTarget1HeightMeters = 0.540;
    public static final double kTarget2HeightMeters = 0.522;
    public static final double kTarget3HeightMeters = 0.530;
    public static final double kTarget4HeightMeters = 0.0;
    public static final double kTarget5HeightMeters = 0.0;
    public static final double kTarget6HeightMeters = 0.0;
    public static final double kTarget7HeightMeters = 0.0;
    public static final double kTarget8HeightMeters = 0.0;

    public static double findTargetHeight(int targetID) {
      switch(targetID) {
        case 1: targetID = 1;
          return kTarget1HeightMeters;
        case 2: targetID = 2;
          return kTarget2HeightMeters;
        case 3: targetID = 3;
          return kTarget3HeightMeters;
        case 4: targetID = 4;
          return kTarget4HeightMeters;
        case 5: targetID = 5;
          return kTarget5HeightMeters;
        case 6: targetID = 6;
          return kTarget6HeightMeters;
        case 7: targetID = 7;
          return kTarget7HeightMeters;
        case 8: targetID = 8;
          return kTarget8HeightMeters;
        default: throw new Error("Invalid Target ID");
      }
    }
  }


  public static final class OIConstants {
    public static final int kDriverJoystickPort = 0;

    public static final int kArcadeDriveSpeedAxis = 1;
    public static final int kArcadeDriveTurnAxis = 3;
  }
}
