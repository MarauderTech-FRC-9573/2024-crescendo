package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.ShooterSubsystem;

public class IntakeSource extends Command {
    ShooterSubsystem shooterSubsystem;

    public IntakeSource(ShooterSubsystem shooterSubsystem) {
        this.shooterSubsystem = shooterSubsystem;
        addRequirements(shooterSubsystem);
    }

    @Override
    public void initialize() {
        shooterSubsystem.setLaunchWheel(ShooterConstants.kIntakeLauncherSpeed);
        shooterSubsystem.setFeedWheel(ShooterConstants.kIntakeFeederSpeed);
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
