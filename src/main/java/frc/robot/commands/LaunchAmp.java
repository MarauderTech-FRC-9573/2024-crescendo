package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
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
        shooterSubsystem.setLaunchWheel(ShooterConstants.kSpeakerAmpLaunchSpeed);
        new WaitCommand(ShooterConstants.kLauncherDelay);
        shooterSubsystem.setFeedWheel(ShooterConstants.kSpeakerAmpLaunchFeederSpeed);
        
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
