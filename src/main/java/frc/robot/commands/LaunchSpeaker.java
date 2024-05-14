package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.ShooterSubsystem;

public class LaunchSpeaker extends Command{
    ShooterSubsystem shooterSubsystem;

 public LaunchSpeaker(ShooterSubsystem shooterSubsystem) {
        this.shooterSubsystem = shooterSubsystem;
        addRequirements(shooterSubsystem);
    }

    @Override
    public void initialize() {
    new SequentialCommandGroup(
        new InstantCommand(() -> shooterSubsystem.setLaunchWheel(ShooterConstants.kSpeakerLaunchFeederSpeed), shooterSubsystem),
        new WaitCommand(ShooterConstants.kLauncherDelay),
        new InstantCommand(() -> shooterSubsystem.setFeedWheel(ShooterConstants.kSpeakerLaunchFeederSpeed), shooterSubsystem)
    ).schedule();
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
