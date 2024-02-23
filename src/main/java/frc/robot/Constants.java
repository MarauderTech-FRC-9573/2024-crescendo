// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;

public final class Constants {
  public static final class DriveConstants {
<<<<<<< HEAD
=======

      // PWM ports/CAN IDs for motor controllers
      public static final int kLeftRearID = 5;
      public static final int kLeftFrontID = 4;
      public static final int kRightRearID = 3;
      public static final int kRightFrontID = 2;
  
      // Current limit for drivetrain motors
      public static final int kCurrentLimit = 60;
  
    public static final int joystickPort = 0;    
>>>>>>> main
    
    // PWM ports/CAN IDs for motor controllers
    public static final int kLeftRearID = 5;
    public static final int kLeftFrontID = 4;
    public static final int kRightRearID = 3;
    public static final int kRightFrontID = 2;
    
    // Current limit for drivetrain motors
    public static final int kCurrentLimit = 60;
    
    public static int operatorControllerPort = 1;
    public static int driveControllerPort = 0;
    
    // values are related to PID and need to be tuned:
    public static final double kP = 0.5;
    public static final double kI = 0.5;
    public static final double kD = 0.1;
    public static final double kDriveTick2Feet = 1.0 / 128 * 6 * Math.PI / 12;
    
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
    
    public static final LinearSystem<N2, N2, N2> kDrivetrainPlant = LinearSystemId.identifyDrivetrainSystem(kvVoltSecondsPerMeter, kaVoltSecondsSquaredPerMeter,kvVoltSecondsPerRadian, kaVoltSecondsSquaredPerRadian);    
    
    public static final int[] kLeftLeadEncoderPorts = new int[] {4, 5};
    public static final int[] kRightLeadEncoderPorts = new int[] {6, 7};
    public static final boolean kLeftEncoderReversed = false;
    public static final boolean kRightEncoderReversed = true;
    
    public static final int kEncoderCPR = 1024;
    public static final double kEncoderDistancePerPulse = (kWheelDiameterMeters * Math.PI) / (double) kEncoderCPR; // Assumes the encoders are directly mounted on the wheel shafts 
  }
  
  public static final class ShooterConstants {
    
<<<<<<< HEAD
    public static double kIntakeLauncherSpeed = 3.0;
    public static double kIntakeFeederSpeed = 3.0;
    public static int launchWheelPort = 5;
    public static int intakeWheelPort = 6;
    public static double kLaunchWheelSpeed;
    public static double kFeedWheelSpeed;
    public static double kLauncherDelay;
    
    
  }
=======
    public static double kIntakeLauncherSpeed = -3.0;
    public static double kIntakeFeederSpeed = -3.0;
    public static int launchWheelPort = 6;
    public static int intakeWheelPort = 7;
    public static double AmpLaunchWheelSpeed = 3.0;
    public static double AmpFeedWheelSpeed = 3.0;
    public static double SpeakerLaunchWheelSpeed = 6.0;
    public static double SpeakerFeedWheelSpeed = 6.0;
>>>>>>> main
  
  
  }
  public static final class OIConstants {    
  }
}