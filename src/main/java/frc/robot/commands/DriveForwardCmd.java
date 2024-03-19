package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;

public class DriveForwardCmd extends Command {
    private DriveSubsystem driveSubsystem;
    private double distance;
    private double speed;
    private Pose2d startPose;
    
    public DriveForwardCmd(DriveSubsystem driveSubsystem, double distance, double speed) {
        this.driveSubsystem = driveSubsystem;
        this.distance = distance;
        this.speed = speed;
        addRequirements(driveSubsystem);
    }
    
    @Override
    public void initialize() {
        
    }
    
    @Override
    public void execute() {
        driveSubsystem.driveArcade(speed, 0);
    }
}
