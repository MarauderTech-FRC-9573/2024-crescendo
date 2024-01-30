package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.CvSink;
import edu.wpi.first.cscore.CvSource;
import edu.wpi.first.cscore.UsbCamera;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

public class VisionSubsystem extends SubsystemBase {
  private final UsbCamera camera;
  private final CvSink cvSink;
  private final CvSource outputStream;

  public VisionSubsystem() {
    camera = CameraServer.startAutomaticCapture();
    camera.setResolution(640, 480);

    cvSink = CameraServer.getVideo();
    outputStream = CameraServer.putVideo("Rectangle", 640, 480);

    // Start the vision thread
    Thread visionThread = new Thread(this::visionProcessing);
    visionThread.start();
  }

  private void visionProcessing() {
    Mat mat = new Mat();

    while (!Thread.interrupted()) {
      if (cvSink.grabFrame(mat) == 0) {
        outputStream.notifyError(cvSink.getError());
        continue;
      }

      // Perform your vision processing here
      processImage(mat);

      outputStream.putFrame(mat);
    }
  }

  private void processImage(Mat mat) {
    // Example: Draw a rectangle on the image
    Imgproc.rectangle(mat, new Point(100, 100), new Point(400, 400),
        new Scalar(255, 255, 255), 5);

    // Add your actual vision processing logic here
    // You can use OpenCV functions to analyze and manipulate the image
    // For example, thresholding, contour detection, etc.
  }

  @Override
  public void periodic() {
    // Add any periodic tasks related to the vision subsystem here
  }
}
