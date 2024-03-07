package frc.robot.subsystems;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
// Simulation libraries
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.simulation.ADXRS450_GyroSim;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import static frc.robot.Constants.DriveConstants.*;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

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

    private final SimpleMotorFeedforward m_feedforward = new SimpleMotorFeedforward(1, 3);
    
    
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
        leftFront.setInverted(true);
        rightFront.setInverted(false);
        
        // Put the front motors into the differential drive object. This will control all 4 motors with
        // the rears set to follow the fronts
        m_drivetrain = new DifferentialDrive(leftFront, rightFront);

        m_odometry = new DifferentialDriveOdometry(Rotation2d.fromDegrees(getHeading()), driveLeftEncoder.getDistance(), driveRightEncoder.getDistance());
    }
    
    /*Method to control the drivetrain using arcade drive. Arcade drive takes a speed in the X (forward/back) direction
    * and a rotation about the Z (turning the robot about it's center) and uses these to control the drivetrain motors */
    public void driveArcade(double speed, double rotation) {
        System.out.println("Speed input to driveArcade: " + speed);
        System.out.println("Rotation input to driveArcade: " + rotation);

        if (Math.floor(speed) == 0 && Math.floor(rotation) == 0) {
            m_drivetrain.arcadeDrive(0, 0);
            System.out.println("No controller input, not moving");
            
        } else {
            System.out.println("Controller input, moving");
            // Calculate the PID output for left and right motors
            System.out.println("LeftEncoder: " + driveLeftEncoder.getRate());
            System.out.println("Right Encoder: " + driveRightEncoder.getRate());

            double leftOutput = leftPIDController.calculate(driveLeftEncoder.getRate(), targetLeftVelocity);
            double rightOutput = rightPIDController.calculate(driveRightEncoder.getRate(), targetRightVelocity);
                        
            // Ensure the motor input is within the allowable range
            leftOutput = MathUtil.clamp(leftOutput, -1.0, 1.0);
            rightOutput = MathUtil.clamp(rightOutput, -1.0, 1.0);

            System.out.println("leftMotorInput Post Clamp: " + leftOutput);
            System.out.println("rightMotorInput Post Clamp: "+ rightOutput);

            // System.out.println("Speed input passed to arcadeDrive: " + speed);
            // System.out.println("Rotation input passed to arcadeDrive: " + rotation);
            
            // System.out.println("Speed argument passed to arcadeDrive: " + (speed + leftOutput));
            // System.out.println("Rotation argument passed to arcadeDrive: " + (rotation + rightOutput)); 
            // Set the motor speeds            
            m_drivetrain.arcadeDrive(speed + leftOutput, rotation + rightOutput);

        }
    }
    
    @Override
    public void periodic() {
        /*This method will be called once per scheduler run. It can be used for running tasks we know we want to update each
        * loop such as processing sensor data. Our drivetrain is simple so we don't have anything to put here */
    }

    public double getHeading() {
        return Math.IEEEremainder(m_gyro.getAngle(), 360) * (DriveConstants.kGyroReversed ? -1.0 : 1.0);
    }

}