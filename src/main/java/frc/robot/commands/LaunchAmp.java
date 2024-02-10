package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ShooterAmpConstants;
import frc.robot.subsystems.ShooterSubsystem;

public class LaunchAmp extends Command {
    ShooterSubsystem shooterSubsystem;

    public LaunchAmp(ShooterSubsystem shooterSubsystem) {
        this.shooterSubsystem = shooterSubsystem;
        addRequirements(shooterSubsystem);
    }

    @Override
    public void initialize() {
        shooterSubsystem.setLaunchWheel(ShooterAmpConstants.kLaunchWheelSpeed);
        shooterSubsystem.setFeedWheel(ShooterAmpConstants.kFeedWheelSpeed);
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
