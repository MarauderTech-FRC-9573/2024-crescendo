package frc.robot.subsystems;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.DriveConstants;
// Simulation libraries
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.simulation.ADXRS450_GyroSim;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.math.MathUtil;
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
    private final Encoder driveLeftEncoder = new Encoder(DriveConstants.kLeftLeadEncoderPorts[0], DriveConstants.kLeftLeadEncoderPorts[1], DriveConstants.kLeftEncoderReversed);
    
    // The right-side drive encoder
    private final Encoder driveRightEncoder = new Encoder(DriveConstants.kRightLeadEncoderPorts[0], DriveConstants.kRightLeadEncoderPorts[1], DriveConstants.kRightEncoderReversed);
    
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
    
    // Gains are for example purposes only - must be determined for your own
    // robot!
    private final SimpleMotorFeedforward m_feedforward = new SimpleMotorFeedforward(1, 3);

    /** Creates a new DriveSubsystem. */
    public DriveSubsystem() {
        SendableRegistry.addChild(differentialDrive, driveLeftLeadMotor);
        SendableRegistry.addChild(differentialDrive, driveRightLeadMotor);
        
        driveLeftLeadMotor.addFollower(driveLeftFollowerMotor);
        driveRightLeadMotor.addFollower(driveRightFollowerMotor);
        
        driveLeftLeadMotor.setInverted(true);
        driveRightLeadMotor.setInverted(false);
        
        // Sets the distance per pulse for the encoders
        driveLeftEncoder.setDistancePerPulse(DriveConstants.kEncoderDistancePerPulse);
        driveRightEncoder.setDistancePerPulse(DriveConstants.kEncoderDistancePerPulse);
        
        resetEncoders();
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
    
    // resets encoders to read 0 
    public void resetEncoders() {
        driveLeftEncoder.reset();
        driveRightEncoder.reset();
    }
    
    // returns average of two 
    public double getAverageEncoderDistance() {
        return (driveLeftEncoder.getDistance() + driveRightEncoder.getDistance()) / 2.0;
    }
    
    // returns value of left encoder
    public Encoder getLeftEncoder() {
        return driveLeftEncoder;
    }
    
    // returns value of right encoder
    public Encoder getRightEncoder() {
        return driveRightEncoder;
    }
   // Is this needed? 
    public void setMaxOutput(double maxOutput) {
        differentialDrive.setMaxOutput(maxOutput);
    }
    // Edited this PID here because it was missing the feedforward
    public void setSpeeds(DifferentialDriveWheelSpeeds speeds) {

        var leftFeedforward = m_feedforward.calculate(speeds.leftMetersPerSecond);
        var rightFeedforward = m_feedforward.calculate(speeds.rightMetersPerSecond);

        final double leftOutput = leftPIDController.calculate(driveLeftEncoder.getRate(), speeds.leftMetersPerSecond);
        final double rightOutput = rightPIDController.calculate(driveRightEncoder.getRate(), speeds.rightMetersPerSecond);
        
        // System.out.println("leftOutput: " + leftOutput + ", rightOutput: " + rightOutput);
        
        driveLeftLeadMotor.setVoltage(leftOutput + leftFeedforward);
        driveRightLeadMotor.setVoltage(rightOutput + rightFeedforward);
    }
    
    /**
    * Drives the robot with the given linear velocity and angular velocity.
    *
    * xSpeed Linear velocity in m/s.
    * rot Angular velocity in rad/s.
    */
    public void drive(double xSpeed, double rot) {
        var wheelSpeeds = m_kinematics.toWheelSpeeds(new ChassisSpeeds(xSpeed, 0.0, rot));
        setSpeeds(wheelSpeeds);
    }
    
    public void arcadeDrive(double speed, double rotation, CommandXboxController controller) {
        double xSpeed = MathUtil.clamp(-controller.getLeftY(), -1.0, 1.0); // Forward/Backward speed
        double ySpeed = MathUtil.clamp(controller.getLeftX(), -1.0, 1.0);  // Turning speed
        
        // Debugging
        // System.out.println("xSpeed: " + xSpeed + ", ySpeed: " + ySpeed);
        
        this.drive(xSpeed, ySpeed);
        differentialDrive.feed();
        
    }
    
    public double getHeading() {
        return Math.IEEEremainder(m_gyro.getAngle(), 360) * (false ? -1.0 : 1.0);
    }

    // Added odometry related stuff here and moved periodic function here

    /** Update robot odometry. */
  public void updateOdometry() {
    m_odometry.update(
        m_gyro.getRotation2d(), driveLeftEncoder.getDistance(), driveRightEncoder.getDistance());
  }

    /** Resets robot odometry. */
    public void resetOdometry(Pose2d pose) {
        resetEncoders();
        m_drivetrainSimulator.setPose(pose);
        m_odometry.resetPosition(
            m_gyro.getRotation2d(), driveLeftEncoder.getDistance(), driveRightEncoder.getDistance(), pose);
  }

    /** Check the current robot pose. */
    public Pose2d getPose() {
        return m_odometry.getPoseMeters();
    }

    /** Update our simulation. This should be run every robot loop in simulation. */
    public void simulationPeriodic() {
        // To update our simulation, we set motor voltage inputs, update the
        // simulation, and write the simulated positions and velocities to our
        // simulated encoder and gyro. We negate the right side so that positive
        // voltages make the right side move forward.
        m_drivetrainSimulator.setInputs(
            driveLeftLeadMotor.get() * RobotController.getInputVoltage(),
            driveRightLeadMotor.get() * RobotController.getInputVoltage());
        m_drivetrainSimulator.update(0.02);

        simLeftEncoder.setDistance(m_drivetrainSimulator.getLeftPositionMeters());
        simLeftEncoder.setRate(m_drivetrainSimulator.getLeftVelocityMetersPerSecond());
        simRightEncoder.setDistance(m_drivetrainSimulator.getRightPositionMeters());
        simRightEncoder.setRate(m_drivetrainSimulator.getRightVelocityMetersPerSecond());
        m_gyroSim.setAngle(-m_drivetrainSimulator.getHeading().getDegrees());
    }

    /** Update odometry - this should be run every robot loop. */
    @Override
    public void periodic()  {
        updateOdometry();
        m_fieldSim.setRobotPose(m_odometry.getPoseMeters());
    }

}



