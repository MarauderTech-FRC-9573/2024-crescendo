package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;

public class DriveForwardCmd extends Command {
    private final DriveSubsystem driveSubsystem;
    private final double distance;

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
        driveSubsystem.setMotors(0.5, 0.5);
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
