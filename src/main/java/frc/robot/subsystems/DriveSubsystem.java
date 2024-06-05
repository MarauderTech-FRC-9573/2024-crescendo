package frc.robot.subsystems;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
// Simulation libraries
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.AnalogGyro;
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
    private final Encoder m_leftEncoder = new Encoder(DriveConstants.kLeftLeadEncoderPorts[0], DriveConstants.kLeftLeadEncoderPorts[1], DriveConstants.kLeftEncoderReversed);
    private final Encoder m_rightEncoder = new Encoder(DriveConstants.kRightLeadEncoderPorts[0], DriveConstants.kRightLeadEncoderPorts[1], DriveConstants.kRightEncoderReversed);
    
    private final PIDController m_leftPIDController = new PIDController(DriveConstants.kP, DriveConstants.kI, DriveConstants.kD);
    private final PIDController m_rightPIDController = new PIDController(DriveConstants.kP, DriveConstants.kI, DriveConstants.kD); 
    
    private final AnalogGyro m_gyro = new AnalogGyro(0);
    
    private final SimpleMotorFeedforward m_feedforward = new SimpleMotorFeedforward(DriveConstants.ksVolts, DriveConstants.kvVoltSecondsPerMeter, DriveConstants.kaVoltSecondsSquaredPerMeter);
    
    private final DifferentialDriveKinematics m_kinematics = new DifferentialDriveKinematics(DriveConstants.kTrackWidth);
    
    private final CANSparkMax leftFront = new CANSparkMax(kLeftFrontID, CANSparkLowLevel.MotorType.kBrushed);
    private final CANSparkMax leftRear = new CANSparkMax(kLeftRearID, CANSparkLowLevel.MotorType.kBrushed);
    private final CANSparkMax rightFront = new CANSparkMax(kRightFrontID, CANSparkLowLevel.MotorType.kBrushed);
    private final CANSparkMax rightRear = new CANSparkMax(kRightRearID, CANSparkLowLevel.MotorType.kBrushed);
    
    private final DifferentialDriveOdometry m_odometry;
    
    public DriveSubsystem() {
        // Set the rear motors to follow the front motors.
        leftRear.follow(leftFront);
        rightRear.follow(rightFront);
        
        // This block of code initializes the DriveSubsystem class. It sets up the rear motors to follow the front motors, sets the current limit for all motors, inverts the left side of the drivetrain, creates a DifferentialDrive object to control all four motors, and sets the maximum output speed.
        leftFront.setSmartCurrentLimit(kCurrentLimit);
        leftRear.setSmartCurrentLimit(kCurrentLimit);
        rightFront.setSmartCurrentLimit(kCurrentLimit);
        rightRear.setSmartCurrentLimit(kCurrentLimit);
        
        // Invert the left side so both side drive forward with positive motor outputs
        leftFront.setInverted(false);
        rightFront.setInverted(true);
        
        // Put the front motors into the differential drive object. This will control all 4 motors with
        // the rears set to follow the fronts
        m_drivetrain = new DifferentialDrive(leftFront, rightFront);
        
        m_leftEncoder.setDistancePerPulse(2 * Math.PI * DriveConstants.kWheelRadius / DriveConstants.kEncoderResolution);
        m_rightEncoder.setDistancePerPulse(2 * Math.PI * DriveConstants.kWheelRadius / DriveConstants.kEncoderResolution);
        
        m_leftEncoder.reset();
        m_rightEncoder.reset();
        
        m_odometry = new DifferentialDriveOdometry(m_gyro.getRotation2d(), m_leftEncoder.getDistance(), m_rightEncoder.getDistance());
        
        setMaxOutput(DriveConstants.maxSpeed);
    }
    
    
    /**
     * Drives the robot using arcade drive control.
     * 
     * @param fwd The forward input value.
     * @param rot The rotation input value.
     */
    public void driveArcade(double fwd, double rot) {
        var speed = m_kinematics.toWheelSpeeds(new ChassisSpeeds(fwd, 0.0, rot));
        
        final double leftFeedforward = m_feedforward.calculate(speed.leftMetersPerSecond);
        final double rightFeedForward = m_feedforward.calculate(speed.rightMetersPerSecond);

        final double leftOutput = m_leftPIDController.calculate(m_leftEncoder.getRate(), speed.leftMetersPerSecond);
        final double rightOutput = m_rightPIDController.calculate(m_rightEncoder.getRate(), speed.rightMetersPerSecond);
        leftFront.setVoltage(leftOutput + leftFeedforward);
        rightFront.setVoltage(rightOutput + rightFeedForward);
    }
    
    
    
    @Override
    public void periodic() {
        // Get the rotation of the robot from the gyro.
        m_odometry.update(m_gyro.getRotation2d(), m_leftEncoder.getDistance(), m_rightEncoder.getDistance());    
    }
        
    public void setMaxOutput(double maxOutput) {
        
        m_drivetrain.setMaxOutput(maxOutput);
        
    }
    
    
}