package frc.robot.subsystems;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class VisionSubsystem extends SubsystemBase {

  public static final double LINEAR_P = 0.1;
  public static final double LINEAR_D = 0.0;
  public static PIDController forwardController = new PIDController(LINEAR_P, 0, LINEAR_D);

  public static final double ANGULAR_P = 0.1;
  public static final double ANGULAR_D = 0.0;
  public static PIDController turnController = new PIDController(ANGULAR_P, 0, ANGULAR_D);

  // Constants such as camera and target height stored. Change per robot and goal!
  final double CAMERA_HEIGHT_METERS = Units.inchesToMeters(24);
  final double TARGET_HEIGHT_METERS = Units.feetToMeters(5);
  // Angle between horizontal and the camera.
  final double CAMERA_PITCH_RADIANS = Units.degreesToRadians(0);
  
  // How far from the target we want to be
  final double GOAL_RANGE_METERS = Units.feetToMeters(3);
  
  // Change this to match the name of your camera as shown in the web UI
  PhotonCamera camera = new PhotonCamera("photonvision");
  

  public PhotonPipelineResult getLatestResult() {
    return camera.getLatestResult();
  }

  public void getAprilTags() {
    var result = camera.getLatestResult();
    if (result.hasTargets()) {
        for (var target : result.getTargets()) {
            System.out.println("Detected April tag: " + target);
        }
    }
}


  
}
