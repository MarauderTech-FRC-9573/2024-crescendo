/*
 * For Shooter Subsystem
 */

package frc.robot.commands;
import static frc.robot.Constants.ShooterConstants;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSubsystem;

public class PrepareLaunchSpeaker extends Command {
    ShooterSubsystem launcher;

    public PrepareLaunchSpeaker(ShooterSubsystem launcher) {
        this.launcher = launcher;
        addRequirements(launcher);
    }

    @Override 
    public void initialize() {
        launcher.setLaunchWheel(ShooterConstants.kSpeakerLauncherSpeed);
    }

    @Override 
    public void execute() {

    }

    @Override 
    public void end(boolean interrupted) {

    }

    @Override 
    public boolean isFinished() {
        return false;
    }
}
