package frc.robot.subsystems;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.DriveConstants;
// Simulation libraries
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.MutableMeasure;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import static frc.robot.Constants.DriveConstants.*;

import static edu.wpi.first.units.MutableMeasure.mutable;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Volts;

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
    
    final SysIdRoutine m_sysIdRoutine;

    CANSparkMax leftFront;
    CANSparkMax leftRear;
    CANSparkMax rightFront;
    CANSparkMax rightRear;
    
    // Gains must be determined, disabled because that's what causes it to spin weirdly
    // private final SimpleMotorFeedforward m_feedforward = new SimpleMotorFeedforward(1, 3);
    
    
    /*Constructor. This method is called when an instance of the class is created. This should generally be used to set up
    * member variables and perform any configuration or set up necessary on hardware.
    */
    public DriveSubsystem() {
        leftFront = new CANSparkMax(kLeftFrontID, CANSparkLowLevel.MotorType.kBrushed);
        leftRear = new CANSparkMax(kLeftRearID, CANSparkLowLevel.MotorType.kBrushed);
        rightFront = new CANSparkMax(kRightFrontID, CANSparkLowLevel.MotorType.kBrushed);
        rightRear = new CANSparkMax(kRightRearID, CANSparkLowLevel.MotorType.kBrushed);
        
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
        rightFront.setInverted(false);
        
        // Put the front motors into the differential drive object. This will control all 4 motors with
        // the rears set to follow the fronts
        m_drivetrain = new DifferentialDrive(leftFront, rightFront);
        
        m_odometry = new DifferentialDriveOdometry(m_gyro.getRotation2d(), driveLeftEncoder.getDistance(), driveRightEncoder.getDistance(), new Pose2d(5.0, 13.5, new Rotation2d()));
        
        // Mutable holder for unit-safe voltage values, persisted to avoid reallocation.
        final MutableMeasure<Voltage> m_appliedVoltage = mutable(Volts.of(0));
        // Mutable holder for unit-safe linear distance values, persisted to avoid reallocation.
        final MutableMeasure<Distance> m_distance = mutable(Meters.of(0));
        // Mutable holder for unit-safe linear velocity values, persisted to avoid reallocation.
        final MutableMeasure<Velocity<Distance>> m_velocity = mutable(MetersPerSecond.of(0));
        
        // Create a new SysId routine for characterizing the drive.
        m_sysIdRoutine =
            // Create the SysId routine
            new SysIdRoutine(
                new SysIdRoutine.Config(),

                new SysIdRoutine.Mechanism(

                    (Measure<Voltage> volts) -> {

                        this.driveTankVolts(volts.in(Volts), volts.in(Volts));

                    },

                null, // No log consumer, since data is recorded by URCL
                    this
                )
            );
    }
    
    boolean isStopped = false;
    
    /*Method to control the drivetrain using arcade drive. Arcade drive takes a speed in the X (forward/back) direction
    * and a rotation about the Z (turning the robot about it's center) and uses these to control the drivetrain motors */
    public void driveArcade(double speed, double rotation) {
        System.out.println("Speed input to driveArcade: " + speed);
        System.out.println("Rotation input to driveArcade: " + rotation);
        
        if (Math.floor(speed) == 0 && Math.floor(rotation) == 0) {
            
            if (isStopped)  {
                System.out.println("Controller input, not moving");
            } else {
                m_drivetrain.arcadeDrive(0, 0);
                System.out.println("No controller input, not moving");
                isStopped = true;
            }
        } else {
            isStopped = false;
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
    
    //Drive using volts for robot characterization

    public void driveTankVolts(double leftVolts, double rightVolts) {
        // System.out.println("Left Volts: " + leftVolts);
        // System.out.println("Right Volts: " + rightVolts);
        m_drivetrain.tankDrive(leftVolts, rightVolts);
        m_drivetrain.feed();
    
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
        // System.out.println("Gyro: " + this.getHeading());
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
    
    /**
    * Returns a command that will execute a quasistatic test in the given direction.
    *
    * @param direction The direction (forward or reverse) to run the test in
    */
    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return m_sysIdRoutine.quasistatic(direction);
    }
    
    /**
    * Returns a command that will execute a dynamic test in the given direction.
    *
    * @param direction The direction (forward or reverse) to run the test in
    */
    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return m_sysIdRoutine.dynamic(direction);
    }
    
}
