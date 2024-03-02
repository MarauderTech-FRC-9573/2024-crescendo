package frc.robot.subsystems;

import static frc.robot.Constants.ShooterConstants;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;
//Almost same code as the shootersubsystem. Incomplete
public class ShooterSubsystem extends SubsystemBase {
  CANSparkMax m_brushMotor;
  CANSparkMax m_intakeMotor;

  public ShooterSubsystem() {
    m_brushMotor = new CANSparkMax(IntakeConstants.brushMotorPort, CANSparkLowLevel.MotorType.kBrushed);
    m_intakeMotor = new CANSparkMax(IntakeConstants.intakeMotorPort, CANSparkLowLevel.MotorType.kBrushed);

  public void setBrushMotor(double speed) {
    m_brushMotor.set(speed);
  }


  public void setIntakeMotor(double speed) {
    m_intakeMotor.set(speed);
  }
  
  public void stop() {
    m_brushMotor.set(0);
    m_intakeMotor.set(0);
  }
}