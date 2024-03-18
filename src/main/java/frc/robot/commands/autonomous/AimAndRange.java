package frc.robot.commands.autonomous;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.VisionSubsystem;

public class AimAndRange extends Command{
    double forwardSpeed;
    double rotationSpeed;
    
    DriveSubsystem m_drivetrain;
    VisionSubsystem m_camera;
    PhotonCamera m_photonCamera;
    PIDController forwardController;
    PIDController turnController; // Declare turnController variable
    
    public AimAndRange(VisionSubsystem visionSubsystem, DriveSubsystem driveSubsystem) {
        m_camera = visionSubsystem;
        m_drivetrain = driveSubsystem;
        m_photonCamera = m_camera.camera;
        forwardController = m_camera.forwardController;
        turnController = m_camera.turnController; 
        addRequirements(m_drivetrain, m_camera);
    }

    @Override
    public void initialize() {
    }
    
    @Override 
    public void execute() {                
        PhotonPipelineResult result = m_camera.getLatestResult();
        
        if (result.hasTargets()) {
            System.out.println("Target detected");
            // First calculate range
            double range =
            PhotonUtils.calculateDistanceToTargetMeters(0.76, 57.13, ((2*(22/7))/9), Units.degreesToRadians(result.getBestTarget().getPitch()));
            
            // Use this range as the measurement we give to the PID controller.
            // -1.0 required to ensure positive PID controller effort _increases_ range
            double forwardSpeed = -forwardController.calculate(range, Units.feetToMeters(3)); // ???????
            
            // Also calculate angular power
            // -1.0 required to ensure positive PID controller effort _increases_ yaw
            double rotationSpeed = -turnController.calculate(result.getBestTarget().getYaw(), 0);
            
            m_drivetrain.driveArcade(forwardSpeed, rotationSpeed);

        } else {
            System.out.println("No target detected");
            m_drivetrain.driveArcade(0, 0);
        }
        
    }
    
    
    @Override 
    public boolean isFinished() {
        return false;
    }
    
    @Override 
    public void end(boolean interrupted) {
    }
}
