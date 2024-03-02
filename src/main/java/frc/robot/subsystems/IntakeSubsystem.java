package frc.robot.subsystems;

import static frc.robot.Constants.ShooterConstants;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;
//Almost same code as the shootersubsystem.
public class IntakeSubsystem extends SubsystemBase {
  CANSparkMax m_brushMotor;
  CANSparkMax m_intakeMotor;

  public IntakeSubsystem() {
    m_brushMotor = new CANSparkMax(IntakeConstants.brushMotorPort, CANSparkLowLevel.MotorType.kBrushed);
    m_intakeMotor = new CANSparkMax(IntakeConstants.intakeMotorPort, CANSparkLowLevel.MotorType.kBrushed);
    Boolean a = true;
    Boolean intakeMotor = a;
    public Command moveIntakeCommand() {
      return this.startEnd(
  /* New command in the IntakeSubsystem instead of a new command in the commands
   * Supposed to have the ground intake start on the ground first or IntakeMotor is true.
   * When the command is called, it checks if the intake is on the ground or up with a true(On the ground) or false(Up to the shooter).
   * Then sets the motor to the speed to move forward or back depending on the true or false statement for a specific amount of time.
   * When the command ends, the intake should be up near the shooter so it sets IntakeMotor to false
   * Need to check for errors
   */
          () -> {
            If (intakeMotor = true) {
              setIntakeMotor(-IntakeConstants.IntakeMotorMoveBack).withTimeout(1);
            } else if (intakeMotor = false) {
              setIntakeMotor(-IntakeConstants.IntakeMotorMoveForward).withTimeout(1);
            }
          },
  
          () -> {
            Boolean intakeMotor = not(a);
            Boolean a = intakeMotor;
            stop();
          });
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
}
}