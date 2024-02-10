package frc.robot.subsystems;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.ShooterConstants;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterSubsystem extends SubsystemBase {}
    PWMSparkMax launchWheel;
    PWMSparkMax intakeWheel;

    public ShooterSubsystem() {
        launchWheel = new PWMSparkMax(ShooterConstants.launchWheelPort);
        intakeWheel = new PWMSparkMax(ShooterConstants.intakeWheelPort);
    }

public class LaunchNote extends Command {
   PWMLauncher m_launcher;

public LaunchNote(PWMLauncher launcher) {
    m_launcher = launcher;


    
    addRequirements(m_launcher);
}

    public Command getIntake() {
        return this.startEnd(
            () -> {
                setFeedWheel(ShooterConstants.kIntakeFeederSpeed);
                setLaunchWheel(ShooterConstants.kIntakeLauncherSpeed);
            }, () -> {
                stop();
            });
    }

@Override
public void initialize() {
    m_launcher.setLaunchWheel(kLauncherSpeed);
    m_launcher.setFeedWheel(kLaunchFeederSpeed);
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
    m_launcher.stop();
}

    public void setLaunchWheel(double speed) {
        launchWheel.set(speed);
    }

    public void setFeedWheel(double speed) {
        intakeWheel.set(speed);
    }

    public void stop() {
        launchWheel.set(0);
        intakeWheel.set(0);
    }

    @Override
    public void periodic() {}
}
