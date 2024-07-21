package frc.robot.subsystems;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;

public class ShooterSubsystem extends SubsystemBase {
  CANSparkMax m_launchWheel;
  CANSparkMax m_feedWheel;

  public ShooterSubsystem() {
    m_launchWheel = new CANSparkMax(ShooterConstants.LauncherPort, CANSparkLowLevel.MotorType.kBrushed);
    m_feedWheel = new CANSparkMax(ShooterConstants.FeedPort, CANSparkLowLevel.MotorType.kBrushed);

    m_launchWheel.setSmartCurrentLimit(ShooterConstants.kLauncherCurrentLimit);
    m_feedWheel.setSmartCurrentLimit(ShooterConstants.kFeedCurrentLimit);

  }

  public Command getIntakeCommand() {
    return this.startEnd(

        () -> {
          setFeedWheel(ShooterConstants.kIntakeFeederSpeed);
          setLaunchWheel(ShooterConstants.kIntakeLauncherSpeed);
        },

        () -> {
          stop();
        });
  }
  public Command getLaunchCommand() {
    return this.startEnd(

        () -> {
          setFeedWheel(ShooterConstants.kSpeakerLaunchFeederSpeed);
          setLaunchWheel(ShooterConstants.kSpeakerLaunchFeederSpeed);
        },

        () -> {
          stop();
        });
  }

  public void setLaunchWheel(double speed) {
    m_launchWheel.set(speed);
  }


  public void setFeedWheel(double speed) {
    m_feedWheel.set(speed);
  }
  
  public void stop() {
    m_launchWheel.set(0);
    m_feedWheel.set(0);
  }
}