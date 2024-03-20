package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberConstants;

public class ClimberSubsystem extends SubsystemBase {
  private final CANSparkMax climberMotor1;
  private final CANSparkMax climberMotor2;
  private boolean isExtended;

  public ClimberSubsystem() {
    climberMotor1 = new CANSparkMax(ClimberConstants.kClimber1ID, MotorType.kBrushed);
    climberMotor2 = new CANSparkMax(ClimberConstants.kClimber2ID, MotorType.kBrushed);
    isExtended = false;
  }

  public void extend() {
    climberMotor1.set(1);
    climberMotor2.set(1);
    isExtended = true;
  }

  public void retract() {
    climberMotor1.set(-1);
    climberMotor2.set(-1);
    isExtended = false;
  }

  public void stop() {
    climberMotor1.stopMotor();
    climberMotor2.stopMotor();
  }

  public boolean isExtended() {
    return isExtended;
  }

  @Override
  public void periodic() {
    System.out.println("Yes");
  }
}