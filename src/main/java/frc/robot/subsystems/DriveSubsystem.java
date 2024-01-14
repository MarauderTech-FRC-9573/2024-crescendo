package frc.robot.subsystems;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

// Simulation libraries
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.simulation.ADXRS450_GyroSim;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.simulation.EncoderSim;

// Imports useful for both actual drivetrain and Simulation
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import frc.robot.Constants.DriveConstants; // Why wasn't this here before lol


public class DriveSubsystem extends SubsystemBase { 
    // aribitrary values for now, double check each of the channels 
    private Spark driveLeftMotor = new Spark(0);
    private Spark driveRightMotor = new Spark(1);
    private Encoder driveLeftEncoder = new Encoder(2, 3);
    private Encoder driveRightEncoder = new Encoder(4, 5);

    // Odometry class for tracking robot pose
    private final DifferentialDriveOdometry m_odometry = new DifferentialDriveOdometry(
        Rotation2d.fromDegrees(getHeading()), 
        driveLeftEncoder.getDistance(), 
        driveRightEncoder.getDistance());

    // These classes help us simulate our drivetrain
    public DifferentialDrivetrainSim m_drivetrainSimulator;
    private final EncoderSim m_leftEncoderSim;
    private final EncoderSim m_rightEncoderSim;

    // The Field2d class shows the field in the sim GUI
    private final Field2d m_fieldSim = new Field2d();
    SmartDashboard.putData("Field", m_fieldSim);

    private final ADXRS450_GyroSim m_gyroSim;

    private final double kEncoderTick2Meter = 1.0 / 4096.0 * 0.128 * Math.PI; // converting encoder ticks to meters

    public double getEncoderMeters() {
        return (driveLeftEncoder.getDistance() + driveRightEncoder.getDistance()) / 2 * kEncoderTick2Meter; 
    }

    public void setMotors(double leftSpeed, double rightSpeed) {
        driveLeftMotor.set(leftSpeed);
        driveRightMotor.set(-rightSpeed);
    }

    @Override
    public void periodic()  {

        m_fieldSim.setRobotPose(m_odometry.getPoseMeters());

    }

    
  /** Resets the drive encoders to currently read a position of 0. */
  public void resetEncoders() {
    driveLeftEncoder.reset();
    driveRightEncoder.reset();
  }

  /**
   * Gets the average distance of the two encoders.
   *
   * @return the average of the two encoder readings
   */
  public double getAverageEncoderDistance() {
    return (driveLeftEncoder.getDistance() + driveRightEncoder.getDistance()) / 2.0;
  }

  /**
   * Gets the left drive encoder.
   *
   * @return the left drive encoder
   */
  public Encoder getLeftEncoder() {
    return driveLeftEncoder;
  }

  /**
   * Gets the right drive encoder.
   *
   * @return the right drive encoder
   */
  public Encoder getRightEncoder() {
    return driveRightEncoder;
  }

  /**
   * Sets the max output of the drive. Useful for scaling the drive to drive more slowly.
   *
   * @param maxOutput the maximum output to which the drive will be constrained
   */
  public void setMaxOutput(double maxOutput) {
    m_drive.setMaxOutput(maxOutput);
  }

  /** Zeroes the heading of the robot. (We don't have a gyro yet)
  public void zeroHeading() {
    m_gyro.reset();
  } */

  /**
   * Returns the heading of the robot.
   *
   * @return the robot's heading in degrees, from -180 to 180
   */
  public double getHeading() {
    return Math.IEEEremainder(m_gyro.getAngle(), 360) * (DriveConstants.kGyroReversed ? -1.0 : 1.0);
  }
}


}