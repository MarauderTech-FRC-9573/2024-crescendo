package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberConstants;

public class ClimberSubsystem extends SubsystemBase {
  private final CANSparkMax climberMotor;
  private final DigitalInput limitSwitch;
  private double position;

  public ClimberSubsystem() {
    climberMotor = new CANSparkMax(ClimberConstants.kClimberID, CANSparkMax.MotorType.kBrushless);
    limitSwitch = new DigitalInput(ClimberConstants.kClimberLimitSwitch);
    position = climberMotor.getEncoder().getPosition();
  }

  public void extend() {
    if (!limitSwitch.get()) {
      climberMotor.set(1);
      position = climberMotor.getEncoder().getPosition();
    }
  }

  public void retract() {
    if (limitSwitch.get()) {
      climberMotor.set(-1);
      position = climberMotor.getEncoder().getPosition();
    }
  }

  public void stop() {
    climberMotor.stopMotor();
  }

  public boolean isRetracted() {
    return limitSwitch.get();
  }

  public double getPosition() {
    return position;
  }

  @Override
  public void periodic() {
    System.out.println("Yes");
  }
}