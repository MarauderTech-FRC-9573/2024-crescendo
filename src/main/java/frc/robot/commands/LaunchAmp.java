package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.ShooterSubsystem;

public class LaunchAmp extends Command {
    ShooterSubsystem shooterSubsystem;

    public LaunchAmp(ShooterSubsystem shooterSubsystem) {
        this.shooterSubsystem = shooterSubsystem;
        addRequirements(shooterSubsystem);
    }

    @Override
    public void initialize() {

    new SequentialCommandGroup(
        new InstantCommand(() -> shooterSubsystem.setLaunchWheel(ShooterConstants.kSpeakerAmpLaunchSpeed), shooterSubsystem),
        new WaitCommand(ShooterConstants.kLauncherDelay),
        new InstantCommand(() -> shooterSubsystem.setFeedWheel(ShooterConstants.kSpeakerAmpLaunchFeederSpeed), shooterSubsystem)
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
