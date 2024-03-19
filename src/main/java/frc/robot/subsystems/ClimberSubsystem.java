package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberConstants;

public class ClimberSubsystem extends SubsystemBase {
  private final CANSparkMax climberMotor1;
  private final CANSparkMax climberMotor2;
  private final DigitalInput limitSwitch;
  private double position1;
  private double position2;

  public ClimberSubsystem() {
    climberMotor1 = new CANSparkMax(ClimberConstants.kClimber1ID, CANSparkMax.MotorType.kBrushless);
    climberMotor2 = new CANSparkMax(ClimberConstants.kClimber2ID, CANSparkMax.MotorType.kBrushless);
    limitSwitch = new DigitalInput(ClimberConstants.kClimberLimitSwitch);
    position1 = climberMotor1.getEncoder().getPosition();
  }

  public void extend() {
    if (!limitSwitch.get()) {
      climberMotor1.set(1);
      climberMotor2.set(1);
      position1 = climberMotor1.getEncoder().getPosition();
      position2 = climberMotor2.getEncoder().getPosition();
    }
  }

  public void retract() {
    if (limitSwitch.get()) {
      climberMotor1.set(-1);
      climberMotor2.set(-1);
      position1 = climberMotor1.getEncoder().getPosition();
      position2 = climberMotor2.getEncoder().getPosition();
    }
  }

  public void stop() {
    climberMotor1.stopMotor();
    climberMotor2.stopMotor();
  }

  public boolean isRetracted() {
    return limitSwitch.get();
  }

  public double getPosition() {
    return position1;
  }

  @Override
  public void periodic() {
    System.out.println("Yes");
  }
}