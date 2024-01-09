package frc.robot.subsystems;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class DriveSubsystem extends SubsystemBase { 
    // aribitrary values for now, double check each of the channels 
    private Spark driveLeftMotor = new Spark(0);
    private Spark driveRightMotor = new Spark(1);
    private Encoder driveLeftEncoder = new Encoder(0, 1);
    private Encoder driveRightEncoder = new Encoder(2, 3);

    private final double kEncoderTick2Meter = 1.0 / 4096.0 * 0.128 * Math.PI; // converting encoder ticks to meters

    public double getEncoderMeters() {
        return (driveLeftEncoder.getDistance() + driveRightEncoder.getDistance()) / 2 * kEncoderTick2Meter; 
    
    }

    public void setMotors(double leftSpeed, double rightSpeed) {
        driveLeftMotor.set(leftSpeed);
        driveRightMotor.set(-rightSpeed);
    }

}