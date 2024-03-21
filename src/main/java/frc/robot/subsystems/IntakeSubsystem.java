package frc.robot.subsystems;

import static frc.robot.Constants.ShooterConstants;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel;
import frc.robot.Constants.IntakeConstants;
import edu.wpi.first.wpilibj2.command.*;

//Almost same code as the shootersubsystem.
public class IntakeSubsystem extends SubsystemBase {
  CANSparkMax m_IntakeMotor;
  CANSparkMax m_ArmMotor;
  Boolean intakeMotor;
  double positionArm;
  
  public IntakeSubsystem() {
    m_IntakeMotor = new CANSparkMax(IntakeConstants.IntakeMotorPort, CANSparkLowLevel.MotorType.kBrushless);
    m_ArmMotor = new CANSparkMax(IntakeConstants.ArmMotorPort, CANSparkLowLevel.MotorType.kBrushless);
  }
    
    public void setIntakeMotor(double speed) {
      m_IntakeMotor.set(speed);
    }
    
    public double getArmPostition() {
      return positionArm;
    }
    
    public void setArmMotor(double speed) {
      m_ArmMotor.set(speed);
    }

    public void setArmPosition(double position) {
      m_ArmMotor.getEncoder().setPosition(0.0);
    }
    
    public void stop() {
      m_IntakeMotor.set(0);
      m_ArmMotor.set(0);
    }

    @Override
    public void periodic() { 
      positionArm = m_ArmMotor.getEncoder().getPosition();
      System.out.println("Arm Motor Position: " + positionArm);
    }
  }