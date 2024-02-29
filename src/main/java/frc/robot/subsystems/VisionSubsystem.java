package frc.robot.subsystems;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class VisionSubsystem extends SubsystemBase {
  // Constants such as camera and target height stored. Change per robot and goal!
  final double CAMERA_HEIGHT_METERS = Units.inchesToMeters(24);
  final double TARGET_HEIGHT_METERS = Units.feetToMeters(5);
  // Angle between horizontal and the camera.
  final double CAMERA_PITCH_RADIANS = Units.degreesToRadians(0);
  
  // How far from the target we want to be
  final double GOAL_RANGE_METERS = Units.feetToMeters(3);
  
  // Change this to match the name of your camera as shown in the web UI
  PhotonCamera camera = new PhotonCamera("idk");
  

  public PhotonPipelineResult getLatestResult() {
    return camera.getLatestResult();
  }

  
}
