package frc.robot.subsystems;
import frc.robot.Constants.ShooterConstants;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel;
import frc.robot.Constants.IntakeConstants;
import edu.wpi.first.wpilibj2.command.*;

//Almost same code as the shootersubsystem.
public class IntakeSubsystem extends SubsystemBase {
  CANSparkMax m_brushMotor;
  CANSparkMax m_intakeMotor;
  Boolean intakeMotor;
  
  public IntakeSubsystem() {
    m_brushMotor = new CANSparkMax(IntakeConstants.brushMotorPort, CANSparkLowLevel.MotorType.kBrushless);
    m_intakeMotor = new CANSparkMax(IntakeConstants.intakeMotorPort, CANSparkLowLevel.MotorType.kBrushless);
    intakeMotor = true;
  }
    
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
    
    public Command moveIntake() {
      return new SequentialCommandGroup(
        new InstantCommand(() -> {
      if (intakeMotor) {
        setIntakeMotor(-IntakeConstants.IntakeMotorMoveBack);
      } else {
        setIntakeMotor(-IntakeConstants.IntakeMotorMoveForward);
      }
      
      intakeMotor = !intakeMotor; 
      }, this),
      new WaitCommand(1),
      new InstantCommand(this::stop, this)
      );      
      
    }
  }
