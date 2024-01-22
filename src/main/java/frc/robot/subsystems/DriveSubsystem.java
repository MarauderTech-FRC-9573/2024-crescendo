package frc.robot.subsystems;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;

// Simulation libraries
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.simulation.ADXRS450_GyroSim;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.simulation.EncoderSim;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.RobotBase;

import org.w3c.dom.traversal.DocumentTraversal;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;

public class DriveSubsystem extends SubsystemBase { 
    // The motors on the left side of the drive.
    private final PWMSparkMax driveLeftLeadMotor = new PWMSparkMax(DriveConstants.leftLeadMotorPort);
    private final PWMSparkMax driveRightLeadMotor = new PWMSparkMax(DriveConstants.rightLeadMotorPort);
    
    // The motors on the right side of the drive.
    private final PWMSparkMax driveLeftFollowerMotor = new PWMSparkMax(DriveConstants.leftFollowMotorPort);
    private final PWMSparkMax driveRightFollowerMotor = new PWMSparkMax(DriveConstants.rightFollowMotorPort);

    
    // The robot's drive
    private final DifferentialDrive differentialDrive = new DifferentialDrive(driveLeftLeadMotor::set, driveRightLeadMotor::set);
    
    // The left-side drive encoder
    private final Encoder driveLeadLeftEncoder = new Encoder(DriveConstants.kLeftLeadEncoderPorts[0], DriveConstants.kLeftLeadEncoderPorts[1], DriveConstants.kLeftEncoderReversed);
    
    // The right-side drive encoder
    private final Encoder driveLeadRightEncoder = new Encoder(DriveConstants.kRightLeadEncoderPorts[0], DriveConstants.kRightLeadEncoderPorts[1], DriveConstants.kRightEncoderReversed);

    private final Encoder driveFollowLeftEncoder = new Encoder(DriveConstants.kLeftFollowEncoderPorts[0], DriveConstants.kLeftFollowEncoderPorts[1], DriveConstants.kLeftEncoderReversed);

    private final Encoder driveFollowRightEncoder = new Encoder(DriveConstants.kRightFollowEncoderPorts[0], DriveConstants.kRightFollowEncoderPorts[1], DriveConstants.kLeftEncoderReversed);

    // PID controllers 
    private final PIDController leftPIDController = new PIDController(DriveConstants.kP, DriveConstants.kI, DriveConstants.kD);
    private final PIDController rightPIDController = new PIDController(DriveConstants.kP, DriveConstants.kI, DriveConstants.kD);
    
    // The gyro sensor
    private final ADXRS450_Gyro m_gyro = new ADXRS450_Gyro();
    
    // Odometry class for tracking robot pose
    private final DifferentialDriveOdometry m_odometry;
    
    // These classes help us simulate our drivetrain
    public DifferentialDrivetrainSim m_drivetrainSimulator;
    private final EncoderSim simLeftEncoder;
    private final EncoderSim simRightEncoder;
    private final Field2d m_fieldSim;
    private final ADXRS450_GyroSim m_gyroSim;

    // someone check this 
    private final DifferentialDriveKinematics m_kinematics = new DifferentialDriveKinematics(3.0);
    
    /** Creates a new DriveSubsystem. */
    public DriveSubsystem() {
        SendableRegistry.addChild(differentialDrive, driveLeftLeadMotor);
        SendableRegistry.addChild(differentialDrive, driveRightLeadMotor);
        
        driveLeftLeadMotor.addFollower(driveLeftFollowerMotor);
        driveRightLeadMotor.addFollower(driveRightFollowerMotor);
        
        // We need to invert one side of the drivetrain so that positive voltages
        // result in both sides moving forward. Depending on how your robot's
        // gearbox is constructed, you might have to invert the left side instead.
        // driveRightFollowerMotor.setInverted(true);
        
        // Sets the distance per pulse for the encoders
        driveLeadLeftEncoder.setDistancePerPulse(DriveConstants.kEncoderDistancePerPulse);
        driveLeadRightEncoder.setDistancePerPulse(DriveConstants.kEncoderDistancePerPulse);
        
        resetEncoders();
        m_odometry =
        new DifferentialDriveOdometry(
        Rotation2d.fromDegrees(getHeading()),
        driveLeadLeftEncoder.getDistance(),
        driveLeadRightEncoder.getDistance());
        
        // simulation code 
        if (RobotBase.isSimulation()) { 
            m_drivetrainSimulator =
            new DifferentialDrivetrainSim(
            DriveConstants.kDrivetrainPlant,
            DriveConstants.kDriveGearbox,
            DriveConstants.kDriveGearing,
            DriveConstants.kTrackwidthMeters,
            DriveConstants.kWheelDiameterMeters / 2.0,
            VecBuilder.fill(0, 0, 0.0001, 0.1, 0.1, 0.005, 0.005));
            
            // The encoder and gyro angle sims let us set simulated sensor readings
            simLeftEncoder = new EncoderSim(driveLeadLeftEncoder);
            simRightEncoder = new EncoderSim(driveLeadRightEncoder);
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
        m_fieldSim.setRobotPose(m_odometry.getPoseMeters());
    }
    
    
    // resets encoders to read 0 
    public void resetEncoders() {
        driveLeadLeftEncoder.reset();
        driveLeadRightEncoder.reset();
    }
    
    // returns average of two 
    public double getAverageEncoderDistance() {
        return (driveLeadLeftEncoder.getDistance() + driveLeadRightEncoder.getDistance()) / 2.0;
    }
    
    // returns value of left encoder
    public Encoder getLeftEncoder() {
        return driveLeadLeftEncoder;
    }
    
    // returns value of right encoder
    public Encoder getRightEncoder() {
        return driveLeadRightEncoder;
    }
    
    
    public void setMaxOutput(double maxOutput) {
        differentialDrive.setMaxOutput(maxOutput);
    }
    
    public void setSpeeds(DifferentialDriveWheelSpeeds speeds) {
        
        final double leftOutput = leftPIDController.calculate(driveLeadLeftEncoder.getRate(), speeds.leftMetersPerSecond);
        final double rightOutput = rightPIDController.calculate(driveLeadRightEncoder.getRate(), speeds.rightMetersPerSecond);
        driveLeftLeadMotor.setVoltage(leftOutput);
        driveRightLeadMotor.setVoltage(rightOutput);
    }
    
    /**
    * Drives the robot with the given linear velocity and angular velocity.
    *
    * @param xSpeed Linear velocity in m/s.
    * @param rot Angular velocity in rad/s.
    */
    public void drive(double leftSpeed, double rightSpeed) {
        DifferentialDriveWheelSpeeds wheelSpeeds = new DifferentialDriveWheelSpeeds(leftSpeed, rightSpeed);
        setSpeeds(wheelSpeeds);
    }
    
    public double getHeading() {
        return Math.IEEEremainder(m_gyro.getAngle(), 360) * (false ? -1.0 : 1.0);
    }
}


