package frc.robot.commands;

import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
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
    XboxController m_controller;

    public AimAtTarget(VisionSubsystem visionSubsystem, DriveSubsystem driveSubsystem, XboxController controller) {
        m_vision = visionSubsystem;
        m_drivetrain = driveSubsystem; 
        m_controller = controller;
    }
    
    @Override
    public void initialize() {
    }
    
    @Override 
    public void execute() {
        forwardSpeed = -m_controller.getRightY();

        if (m_controller.getAButton()) {
            // Vision-alignment mode
            // Query the latest result from PhotonVision
            PhotonPipelineResult result = m_vision.getLatestResult();

            if (result.hasTargets()) {
                // Calculate angular turn power
                // -1.0 required to ensure positive PID controller effort _increases_ yaw
                rotationSpeed = -VisionConstants.turnController.calculate(result.getBestTarget().getYaw(), 0);
            } else {
                // If we have no targets, stay still.
                rotationSpeed = 0;
            }
        } else {
            // Manual Driver Mode
            rotationSpeed = m_controller.getLeftX();
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
