package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ShooterSpeakerConstants;
import frc.robot.subsystems.ShooterSubsystem;

public class LaunchSpeaker extends Command{
    ShooterSubsystem shooterSubsystem;

 public LaunchSpeaker(ShooterSubsystem shooterSubsystem) {
        this.shooterSubsystem = shooterSubsystem;
        addRequirements(shooterSubsystem);
    }

    @Override
    public void initialize() {
        shooterSubsystem.setLaunchWheel(ShooterSpeakerConstants.kLaunchWheelSpeed);
        shooterSubsystem.setFeedWheel(ShooterSpeakerConstants.kFeedWheelSpeed);
    }

    @Override 
    public void execute() {

    }

    @Override 
    public boolean isFinished() {
        return false;
    }

    @Override 
    public void end(boolean interrupted) {
        shooterSubsystem.stop();
    }
}
