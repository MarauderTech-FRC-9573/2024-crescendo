package frc.robot.commands;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.CounterBase.EncodingType;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.Constants.DriveConstants;

public class DriveForwardCmd extends Command {
    private Encoder encoder = new Encoder(0, 1, false, EncodingType.k4X);
    
    private final DriveSubsystem driveSubsystem;
    private final double distance;
    
    private final double iLimit = 1.0;    
    double setpoint = 0;
    double errorSum = 0;
    double lastTimestamp = 0;
    double lastError = 0;
    
    
    public DriveForwardCmd(DriveSubsystem driveSubsystem, double distance) {
        this.driveSubsystem = driveSubsystem;
        this.distance = distance;
        addRequirements(driveSubsystem);
    }
    
    @Override 
    public void initialize() {
        setpoint = encoder.get() * DriveConstants.kDriveTick2Feet + distance;
        encoder.reset();
        System.out.println("DriveForward started!");
    }
    
    @Override
    public void execute() {
        double sensorPosition = encoder.get() * DriveConstants.kDriveTick2Feet;
        double error = setpoint - sensorPosition;
        double dt = Timer.getFPGATimestamp() - lastTimestamp;
        
        if (Math.abs(error) < iLimit) {
            errorSum += error * dt;
        }
        
        double errorRate = (error - lastError) / dt;
        double outputSpeed = DriveConstants.kP * error + DriveConstants.kI * errorSum + DriveConstants.kD * errorRate;
        
        driveSubsystem.setMotors(outputSpeed, -outputSpeed);
        
        lastTimestamp = Timer.getFPGATimestamp();
        lastError = error;
    }
    
    @Override
    public void end(boolean interrputed) {
        driveSubsystem.setMotors(0, 0);
        System.out.println("DriveForward done!");
    }
    
    @Override
    public boolean isFinished() {
        if (driveSubsystem.getEncoderMeters() > distance) {
            return true;
        } else {
            return false;
        }
    }
    
}