package frc.robot.subsystems;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
// Simulation libraries
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import static frc.robot.Constants.DriveConstants.*;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel;

/* This class declares the subsystem for the robot drivetrain if controllers are connected via CAN. Make sure to go to
* RobotContainer and uncomment the line declaring this subsystem and comment the line for PWMDrivetrain.
*
* The subsystem contains the objects for the hardware contained in the mechanism and handles low level logic
* for control. Subsystems are a mechanism that, when used in conjuction with command "Requirements", ensure
* that hardware is only being used by 1 command at a time.
*/
public class DriveSubsystem extends SubsystemBase {
    /*Class member variables. These variables represent things the class needs to keep track of and use between
    different method calls. */
    DifferentialDrive m_drivetrain;
    
    // ENCODERS
    private final Encoder driveLeftEncoder = new Encoder(DriveConstants.kLeftLeadEncoderPorts[0], DriveConstants.kLeftLeadEncoderPorts[1], DriveConstants.kLeftEncoderReversed);
    private final Encoder driveRightEncoder = new Encoder(DriveConstants.kRightLeadEncoderPorts[0], DriveConstants.kRightLeadEncoderPorts[1], DriveConstants.kRightEncoderReversed);
    
    // PID
    private double targetLeftVelocity = 3; // Target velocity in meters per second
    private double targetRightVelocity = 3; // Target velocity in meters per second    
    
    // Gyroscope
    private final ADXRS450_Gyro m_gyro = new ADXRS450_Gyro();
    
    private final PIDController leftPIDController = new PIDController(DriveConstants.kP, DriveConstants.kI, DriveConstants.kD);
    private final PIDController rightPIDController = new PIDController(DriveConstants.kP, DriveConstants.kI, DriveConstants.kD);
    
    // ODOMETRY 
    private final DifferentialDriveOdometry m_odometry;
    
    Pose2d m_pose;
    
    
    // Gains must be determined, disabled because that's what causes it to spin weirdly
    // private final SimpleMotorFeedforward m_feedforward = new SimpleMotorFeedforward(1, 3);
    
    
    /*Constructor. This method is called when an instance of the class is created. This should generally be used to set up
    * member variables and perform any configuration or set up necessary on hardware.
    */
    public DriveSubsystem() {
        CANSparkMax leftFront = new CANSparkMax(kLeftFrontID, CANSparkLowLevel.MotorType.kBrushed);
        CANSparkMax leftRear = new CANSparkMax(kLeftRearID, CANSparkLowLevel.MotorType.kBrushed);
        CANSparkMax rightFront = new CANSparkMax(kRightFrontID, CANSparkLowLevel.MotorType.kBrushed);
        CANSparkMax rightRear = new CANSparkMax(kRightRearID, CANSparkLowLevel.MotorType.kBrushed);
        
        /*Sets current limits for the drivetrain motors. This helps reduce the likelihood of wheel spin, reduces motor heating
        *at stall (Drivetrain pushing against something) and helps maintain battery voltage under heavy demand */
        leftFront.setSmartCurrentLimit(kCurrentLimit);
        leftRear.setSmartCurrentLimit(kCurrentLimit);
        rightFront.setSmartCurrentLimit(kCurrentLimit);
        rightRear.setSmartCurrentLimit(kCurrentLimit);
        
        // Set the rear motors to follow the front motors.
        leftRear.follow(leftFront);
        rightRear.follow(rightFront);
        
        
        // Invert the left side so both side drive forward with positive motor outputs
        leftFront.setInverted(false);
        rightFront.setInverted(true);
        
        // Put the front motors into the differential drive object. This will control all 4 motors with
        // the rears set to follow the fronts
        m_drivetrain = new DifferentialDrive(leftFront, rightFront);
        
        m_odometry = new DifferentialDriveOdometry(m_gyro.getRotation2d(), driveLeftEncoder.getDistance(), driveRightEncoder.getDistance(), new Pose2d(5.0, 13.5, new Rotation2d()));
        
        setMaxOutput(DriveConstants.maxSpeed);
    }
    
    boolean isStopped = false;
    
    /*Method to control the drivetrain using arcade drive. Arcade drive takes a speed in the X (forward/back) direction
    * and a rotation about the Z (turning the robot about it's center) and uses these to control the drivetrain motors */
    public void driveArcade(double speed, double rotation) {
        
        double leftOutput = leftPIDController.calculate(driveLeftEncoder.getRate(), targetLeftVelocity);
        double rightOutput = rightPIDController.calculate(driveRightEncoder.getRate(), targetRightVelocity);
        
        // Ensure the motor input is within the allowable range
        leftOutput = MathUtil.clamp(leftOutput, -1.0, 1.0);
        rightOutput = MathUtil.clamp(rightOutput, -1.0, 1.0);

        m_drivetrain.arcadeDrive(speed + leftOutput, -rotation + rightOutput); 
    }
    
    
    
    @Override
    public void periodic() {
        
        // Get the rotation of the robot from the gyro.
        var gyroAngle = m_gyro.getRotation2d();
        
        // Update the pose
        m_pose = m_odometry.update(gyroAngle,
        driveLeftEncoder.getDistance(),
        driveRightEncoder.getDistance());
        SmartDashboard.putNumber("Gyro ", this.getHeading());

    }
    
    public double getHeading() {
        return Math.IEEEremainder(m_gyro.getAngle(), 360) * (DriveConstants.kGyroReversed ? -1.0 : 1.0);
    }
    
    public void zeroHeading() {
        
        m_gyro.reset();
        
    }
    
    public void setMaxOutput(double maxOutput) {
        
        m_drivetrain.setMaxOutput(maxOutput);
        
    }
    
    public double getTurnRate() {
        
        return m_gyro.getRate() * (DriveConstants.kGyroReversed ? -1.0 : 1.0);
        
    }
    
}