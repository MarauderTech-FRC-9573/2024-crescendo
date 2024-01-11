package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.DriveSubsystem;

public class DriveForwardCmd extends Command {
    private final DriveSubsystem driveSubsystem;
    private final double distance;
    private final double kP = 0.5;
    private final double kI= 0.5;
    private final double kD = 0.1;
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
        System.out.println("DriveForward started!");
    }

    @Override
    public void execute() {
        double sensorPosition = encoder.get() * kDriveTick2Feet;
        double error = setpoint - sensorPosition;
        double dt = Timer.getFPGATimestamp() - lastTimestamp;

        if (Math.abs(error) < iLimit) {
            errorSum += error * dt;
        }

        double errorRate = (error - lastError) / dt;
        double outputSpeed = kP * error + kI * errorSum + kD * errorRate;

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
        if (driveSubsystem.getEncoderMeters() > distance)
            return true;
        else
            return false;
    }

}
