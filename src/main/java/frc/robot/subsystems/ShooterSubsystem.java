package frc.robot.subsystems;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.ShooterConstants;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterSubsystem extends SubsystemBase{
    PWMSparkMax launchWheel;
    PWMSparkMax intakeWheel;

    public ShooterSubsystem() {
        launchWheel = new PWMSparkMax(ShooterConstants.launchWheelPort);
        intakeWheel = new PWMSparkMax(ShooterConstants.intakeWheelPort);
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

    public Command shootAmp() {
        return this.startEnd(
            () -> {
                setFeedWheel(ShooterConstants.kAmpFeederSpeed);
                setLaunchWheel(ShooterConstants.kAmpLauncherSpeed);
            }, () -> {
                stop();
            });
    }

    public Command shootSpeaker() {
        return this.startEnd(
            () -> {
                setFeedWheel(ShooterConstants.kSpeakerFeederSpeed);
                setLaunchWheel(ShooterConstants.kSpeakerLauncherSpeed);
            }, () -> {
                stop();
            });
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
