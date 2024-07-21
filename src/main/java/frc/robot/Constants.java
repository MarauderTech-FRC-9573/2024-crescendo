// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;

public final class Constants {
  
  public static final class DriveConstants {
    
    // PWM ports/CAN IDs for motor controllers
    public static final int kLeftRearID = 5;
    public static final int kLeftFrontID = 4;
    public static final int kRightRearID = 3;
    public static final int kRightFrontID = 2;
    
    // Current limit for drivetrain motors
    public static final int kCurrentLimit = 60;
    
    public static final int joystickPort = 0;    
    
    public static int operatorControllerPort = 1;
    public static int driveControllerPort = 0;
    
    // values are related to PID and need to be tuned:
    public static final double kP = 0;
    public static final double kI = 0;
    public static final double kD = 0;
    public static final double kDriveTick2Feet = 1.0 / 128 * 6 * Math.PI / 12;
    
    //PID values for gyro taken from wpilib gyrocommand example
    
    public static final boolean kGyroReversed = false;
    
    public static final double kStabilizationP = 1;
    public static final double kStabilizationI = 0.5;
    public static final double kStabilizationD = 0;
    
    public static final double kTurnP = 0;
    public static final double kTurnI = 0;
    public static final double kTurnD = 0;
    
    public static final double kMaxTurnRateDegPerS = 100;
    public static final double kMaxTurnAccelerationDegPerSSquared = 300;
    
    public static final double kTurnToleranceDeg = 5;
    public static final double kTurnRateToleranceDegPerS = 10; // degrees per second
    
    // varys per robot and must be tuned 
    // try Robot Characterization Toolsuite to get these values
    public static final double ksVolts = 0.22;
    public static final double kvVoltSecondsPerMeter = 1.98;
    public static final double kaVoltSecondsSquaredPerMeter = 0.2;
    public static final double kvVoltSecondsPerRadian = 1.5;
    public static final double kaVoltSecondsSquaredPerRadian = 0.3;
    
    public static final DCMotor kDriveGearbox = DCMotor.getCIM(2);
    public static final double kDriveGearing = 8;
    public static double kTrackwidthMeters = 0.69; 
    public static double kWheelDiameterMeters = 0.15;
    
    // max speed of the robot
    public static double maxSpeed = 1.0;
    
    
    public static final LinearSystem<N2, N2, N2> kDrivetrainPlant = LinearSystemId.identifyDrivetrainSystem(kvVoltSecondsPerMeter, kaVoltSecondsSquaredPerMeter,kvVoltSecondsPerRadian, kaVoltSecondsSquaredPerRadian);    
    
    public static final int[] kLeftLeadEncoderPorts = new int[] {4, 5};
    public static final int[] kRightLeadEncoderPorts = new int[] {6, 7};
    public static final int[] kLeftFollowEncoderPorts = new int[] {8, 9};
    public static final int[] kRightFollowEncoderPorts = new int[] {10, 11};
    
    public static final boolean kLeftEncoderReversed = false;
    public static final boolean kRightEncoderReversed = true;
    
    public static final int kEncoderCPR = 1024;
    public static final double kEncoderDistancePerPulse = (kWheelDiameterMeters * Math.PI) / (double) kEncoderCPR; // Assumes the encoders are directly mounted on the wheel shafts
    
    public static final int kAngleSetpoint = 0;
    
    
  }
  
  
  // public static final class ShooterConstants {
    
    //   public static double kIntakeLauncherSpeed = -3.0;
    //   public static double kIntakeFeederSpeed = -3.0;
    //   public static int launchWheelPort = 6;
    //   public static int intakeWheelPort = 7;
    //   public static double AmpLaunchWheelSpeed = 3.0;
    //   public static double AmpFeedWheelSpeed = 3.0;
    //   public static double SpeakerLaunchWheelSpeed = 6.0;
    //   public static double SpeakerFeedWheelSpeed = 6.0;
    
    
    // }
    
    // Intake Constants
    public static final class IntakeConstants {
      
      // Motor Speeds
      public static double IntakeMotorReleaseSpeed = -0.01;
      public static double IntakeMotorReceiveSpeed = 0.01;
      public static double ArmMotorMoveForwardSpeed = 0.01;
      public static double ArmMotorMoveBackwardSpeed = -0.01;
      
      // Motor ID Ports
      public static int IntakeMotorPort = 9;
      public static int ArmMotorPort = 8;
      
    }
    
    // OI constants
    public static final class OIConstants {    
      
    }
    
    public static class VisionConstants {
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
      
      // PID constants should be tuned per robot
      
      public static final double CAMERA_HEIGHT_METERS = Units.inchesToMeters(24);
      public static final double TARGET_HEIGHT_METERS = Units.feetToMeters(5);
      public static final double GOAL_RANGE_METERS = Units.feetToMeters(3);
      public static double CAMERA_PITCH_RADIANS = Units.degreesToRadians(3);
      
    }
    
    public static class FieldConstants {
      public static final double kFieldLength = 4.0;
      public static final double kFieldWidth = 3.0;
    }
    
    public static final class ShooterConstants {
      
      // PWM ports/CAN IDs for motor controllers
      public static final int LauncherPort= 6; //Bottom motor away from mouth
      public static final int FeedPort = 7; //Top motor near the mouth
      
      // Current limit for launcher and feed wheels
      public static final int kLauncherCurrentLimit = 80;
      public static final int kFeedCurrentLimit = 80;
      
      // Speeds for wheels when intaking and launching. Intake speeds are negative to run the wheels
      // in reverse
      public static final double kSpeakerLauncherSpeed = -1; //Speed of bottom motor
      public static final double kSpeakerLaunchFeederSpeed = -1; //Speed of top motor
      public static final double kSpeakerAmpLauncherSpeed = -0.3; //Speed of bottom motor
      public static final double kSpeakerAmpLaunchFeederSpeed = -0.4; //Speed of top motor
      public static final double kIntakeLauncherSpeed = 0.8; //Speed of bottom motor
      public static final double kIntakeFeederSpeed = 0.2; //Speed of top motor
      
      public static final double kLauncherDelay = 1;
      
      
    }
    
    
  }