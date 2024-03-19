package frc.robot.subsystems;

import static frc.robot.Constants.ShooterConstants;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel;
import frc.robot.Constants.IntakeConstants;
import edu.wpi.first.wpilibj2.command.*;

//Almost same code as the shootersubsystem.
public class IntakeSubsystem extends SubsystemBase {
  CANSparkMax m_IntakeMotor;
  CANSparkMax m_ArmMotor;
  Boolean intakeMotor;
  
  public IntakeSubsystem() {
    m_IntakeMotor = new CANSparkMax(IntakeConstants.IntakeMotorPort, CANSparkLowLevel.MotorType.kBrushed);
    m_ArmMotor = new CANSparkMax(IntakeConstants.ArmMotorPort, CANSparkLowLevel.MotorType.kBrushed);
    armMotor = true;
    getpositionArm = m_ArmMotor.getEncoder().getPosition();
  }
    
    public void setIntakeMotor(double speed) {
      m_IntakeMotor.set(speed);
    }
    
    
    public void setArmMotor(double speed) {
      m_ArmMotor.set(speed);
    }
    
    public void stop() {
      m_IntakeMotor.set(0);
      m_ArmMotor.set(0);
    }
    
    public Command moveArm() {
      return new SequentialCommandGroup(
        new InstantCommand(() -> {
      if (armMotor) {
        setArmMotor(-IntakeConstants.IntakeMotorMoveBack);
        getpositionArm = m_ArmMotor.getEncoder().getPosition();
      } else {
        setArmMotor(-IntakeConstants.IntakeMotorMoveForward);
        getpositionArm = m_ArmMotor.getEncoder().getPosition();
      }
      
      armMotor = !armMotor; 
      }, this),
      new WaitCommand(1),
      new InstantCommand(this::stop, this)
      );      

      public double getpositionArm() {
        return getpositionArm;
      }
      
    }
  }
