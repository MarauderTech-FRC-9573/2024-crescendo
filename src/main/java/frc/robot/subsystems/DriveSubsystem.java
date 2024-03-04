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

public class DriveSubsystem extends SubsystemBase {
    // MOTOR CONTROLLERS
    CANSparkMax leftFront;
    CANSparkMax leftRear;
    CANSparkMax rightFront;
    CANSparkMax rightRear;
    
    // ENCODERS
    private final Encoder driveLeftEncoder = new Encoder(DriveConstants.kLeftLeadEncoderPorts[0], DriveConstants.kLeftLeadEncoderPorts[1], DriveConstants.kLeftEncoderReversed);
    private final Encoder driveRightEncoder = new Encoder(DriveConstants.kRightLeadEncoderPorts[0], DriveConstants.kRightLeadEncoderPorts[1], DriveConstants.kRightEncoderReversed);
    
    // PID
    private double targetLeftVelocity = 3; // Target velocity in meters per second
    private double targetRightVelocity = 3; // Target velocity in meters per second
    
    private final PIDController leftPIDController = new PIDController(DriveConstants.kP, DriveConstants.kI, DriveConstants.kD);
    private final PIDController rightPIDController = new PIDController(DriveConstants.kP, DriveConstants.kI, DriveConstants.kD);
    
    // GRYOSCOPE
    private final ADXRS450_Gyro m_gyro = new ADXRS450_Gyro();
    
    // ODOMETRY 
    private final DifferentialDriveOdometry m_odometry;
    
    // SIMULATION
    public DifferentialDrivetrainSim m_drivetrainSimulator;
    private final EncoderSim simLeftEncoder;
    private final EncoderSim simRightEncoder;
    private final Field2d m_fieldSim;
    private final ADXRS450_GyroSim m_gyroSim;
    
    /*Class member variables. These variables represent things the class needs to keep track of and use between
    different method calls. */
    DifferentialDrive m_drivetrain;
    
    private final SimpleMotorFeedforward m_feedforward = new SimpleMotorFeedforward(1, 3);
    
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
        leftFront.setInverted(true);
        rightFront.setInverted(false);
        
        // Put the front motors into the differential drive object. This will control all 4 motors with
        // the rears set to follow the fronts
        m_drivetrain = new DifferentialDrive(leftFront, rightFront);
        
        m_odometry = new DifferentialDriveOdometry(Rotation2d.fromDegrees(getHeading()), driveLeftEncoder.getDistance(), driveRightEncoder.getDistance());
        
        // simulation code 
        if (RobotBase.isSimulation()) { 
            m_drivetrainSimulator = new DifferentialDrivetrainSim(DriveConstants.kDrivetrainPlant, DriveConstants.kDriveGearbox, DriveConstants.kDriveGearing, DriveConstants.kTrackwidthMeters, DriveConstants.kWheelDiameterMeters / 2.0, VecBuilder.fill(0, 0, 0.0001, 0.1, 0.1, 0.005, 0.005));
            
            // The encoder and gyro angle sims let us set simulated sensor readings
            simLeftEncoder = new EncoderSim(driveLeftEncoder);
            simRightEncoder = new EncoderSim(driveRightEncoder);
            m_gyroSim = new ADXRS450_GyroSim(m_gyro);
            
            // the Field2d class lets us visualize our robot in the simulation GUI.
            m_fieldSim = new Field2d();
            SmartDashboard.putData("Field", m_fieldSim);
        } else {
            simLeftEncoder = null;
            simRightEncoder = null;
            m_gyroSim = null;
            m_fieldSim = null;
        }        
    }
    
    @Override
    public void periodic()  {
        
        updateOdometry();
        
        if (m_fieldSim != null) {
            
            m_fieldSim.setRobotPose(m_odometry.getPoseMeters());
            
        }
        
        updateOdometry();
        
        // Calculate the PID output for left and right motors
        double leftOutput = leftPIDController.calculate(driveLeftEncoder.getRate(), targetLeftVelocity);
        double rightOutput = rightPIDController.calculate(driveRightEncoder.getRate(), targetRightVelocity);
        
        // Apply the calculated output to the motors along with feedforward for velocity control
        double leftMotorInput = leftOutput + m_feedforward.calculate(targetLeftVelocity);
        double rightMotorInput = rightOutput + m_feedforward.calculate(targetRightVelocity);

        System.out.println("leftMotorInput: " + leftMotorInput);
        System.out.println("rightMotorInput: "+ rightMotorInput);
        
        // Ensure the motor input is within the allowable range
        leftMotorInput = MathUtil.clamp(leftMotorInput, -1.0, 1.0);
        rightMotorInput = MathUtil.clamp(rightMotorInput, -1.0, 1.0);
        
        // Set the motor speeds
        m_drivetrain.arcadeDrive(leftMotorInput, rightMotorInput);
        
        if (m_fieldSim != null) {
            m_fieldSim.setRobotPose(m_odometry.getPoseMeters());
        }

        System.out.println("Gyro get heading: " + Math.IEEEremainder(m_gyro.getAngle(), 360) * (false ? -1.0 : 1.0));
        
        SmartDashboard.putNumber("Gyro", Math.IEEEremainder(m_gyro.getAngle(), 360) * (false ? -1.0 : 1.0));
        
    }
    
    public double calculateTurningCorrection() {
        return (DriveConstants.kAngleSetpoint - m_gyro.getAngle()) * DriveConstants.kP;
    }
    
    
    public void arcadeDrive(double speed, double rotation) {

        speed = Math.floor(speed);
        rotation = Math.floor(rotation);

        System.out.println("Speed: " + speed);
        System.out.println("Rotation: " + rotation);

        // Stops the robot from moving if there is no controller input, preventing drift 
        //if (speed == 0 && rotation == 0) { 
            //m_drivetrain.arcadeDrive(0,0);
        //} else { 
            //m_drivetrain.arcadeDrive(speed, rotation);
        //}
    }
    
    // Pertaining to odometry and gryo PID
    
    /**
    * Returns the turn rate of the robot.
    *
    * @return The turn rate of the robot, in degrees per second
    */
    public double getTurnRate() {
        return m_gyro.getRate() * (DriveConstants.kGyroReversed ? -1.0 : 1.0);
    }
    
    /** Zeroes the heading of the robot. */
    public void zeroHeading() {
        m_gyro.reset();
    }
    
    public double getHeading() {
        return Math.IEEEremainder(m_gyro.getAngle(), 360) * (DriveConstants.kGyroReversed ? -1.0 : 1.0);
    }
    
    public void updateOdometry() {
        m_odometry.update(m_gyro.getRotation2d(), driveLeftEncoder.getDistance(), driveRightEncoder.getDistance());
    }
    
    /** Resets robot odometry. */
    public void resetOdometry(Pose2d pose) {
        m_drivetrainSimulator.setPose(pose);
        m_odometry.resetPosition(
        m_gyro.getRotation2d(), driveLeftEncoder.getDistance(), driveRightEncoder.getDistance(), pose);
    }
    
    /** Check the current robot pose. */
    public Pose2d getPose() {
        return m_odometry.getPoseMeters();
    }
    
    
    /* 
    * CODE FOR SIMULATION
    */
    
    
    /** Update our simulation. This should be run every robot loop in simulation. */
    public void simulationPeriodic() {
        // To update our simulation, we set motor voltage inputs, update the
        // simulation, and write the simulated positions and velocities to our
        // simulated encoder and gyro. We negate the right side so that positive
        // voltages make the right side move forward.
        m_drivetrainSimulator.setInputs(
        leftFront.get() * RobotController.getInputVoltage(),
        rightFront.get() * RobotController.getInputVoltage());
        m_drivetrainSimulator.update(0.02);
        
        simLeftEncoder.setDistance(m_drivetrainSimulator.getLeftPositionMeters());
        simLeftEncoder.setRate(m_drivetrainSimulator.getLeftVelocityMetersPerSecond());
        simRightEncoder.setDistance(m_drivetrainSimulator.getRightPositionMeters());
        simRightEncoder.setRate(m_drivetrainSimulator.getRightVelocityMetersPerSecond());
        m_gyroSim.setAngle(-m_drivetrainSimulator.getHeading().getDegrees());
    }
    
    // Is this needed? Apparently this is useful for scaling robot speed
    public void setMaxOutput(double maxOutput) {
        m_drivetrain.setMaxOutput(maxOutput);
    }
    
}
