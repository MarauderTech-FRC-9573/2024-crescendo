package frc.robot.commands;

import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.Constants.VisionConstants;

public class AimAtTarget extends Command{
    double forwardSpeed;
    double rotationSpeed;
    
    VisionSubsystem m_vision;
    DriveSubsystem m_drivetrain;
    CommandXboxController m_controller;
    
    public AimAtTarget(VisionSubsystem visionSubsystem, DriveSubsystem driveSubsystem, CommandXboxController controller) {
        m_vision = visionSubsystem;
        m_drivetrain = driveSubsystem; 
        m_controller = controller;
    }
    
    @Override
    public void initialize() {
    }
    
    @Override 
    public void execute() {        
        // Vision-alignment mode
        // Query the latest result from PhotonVision
        PhotonPipelineResult result = m_vision.getLatestResult();
        
        if (result.hasTargets()) {
            // Calculate angular turn power
            // -1.0 required to ensure positive PID controller effort _increases_ yaw
            double range = PhotonUtils.calculateDistanceToTargetMeters(VisionConstants.CAMERA_HEIGHT_METERS, VisionConstants.TARGET_HEIGHT_METERS, VisionConstants.CAMERA_PITCH_RADIANS, Units.degreesToRadians(result.getBestTarget().getPitch()));

            forwardSpeed = -VisionConstants.forwardController.calculate(range, VisionConstants.GOAL_RANGE_METERS);
            rotationSpeed = -VisionConstants.turnController.calculate(result.getBestTarget().getYaw(), 0);
            System.out.println("TARGET DETECTED");

        } else {
            // If we have no targets, stay still.
            forwardSpeed = 0;
            rotationSpeed = 0;
            System.out.println("TARGET NOT DETECTED");
        }
        
        // Use our forward/turn speeds to control the drivetrain
        m_drivetrain.arcadeDrive(forwardSpeed, rotationSpeed);
    }
    
    
    @Override 
    public boolean isFinished() {
        return false;
    }
    
    @Override 
    public void end(boolean interrupted) {
    }
}
