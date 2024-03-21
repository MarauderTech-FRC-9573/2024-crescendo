package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.IntakeSubsystem;
// IntakeReceiver is for the brush motors to receive the note from the ground.
/*Basicly the same code from the shootersubsystem with different names.
 *
*/

public class IntakeArmSwitch extends Command {
    IntakeSubsystem intakeSubsystem;
    PIDController pid;
    double setpoint = 0.25;

    public IntakeArmSwitch(IntakeSubsystem intakeSubsystem) {
        this.intakeSubsystem = intakeSubsystem;
        this.pid = intakeSubsystem.pid;
        pid.setTolerance(5, 10);
        addRequirements(intakeSubsystem);
    }

    @Override
    public void initialize() {
        intakeSubsystem.setArmPosition(0.0);
    }

    @Override 
    public void execute() {
        if (intakeSubsystem.getArmPostition() > 0.25) {
            double movement = pid.calculate(intakeSubsystem.getArmPostition(), 0.25);
            System.out.println(movement);
            intakeSubsystem.setArmMotor(movement);
        } else if (intakeSubsystem.getArmPostition() < 0.25) {
            double movement = pid.calculate(intakeSubsystem.getArmPostition(), 0.25);
            System.out.println(movement);
            intakeSubsystem.setArmMotor(movement);
        }
        else {
            intakeSubsystem.stop();
            pid.reset(); 
        }
    }

    @Override 
    public boolean isFinished() {
        return false;
    }

    @Override 
    public void end(boolean interrupted) {
        intakeSubsystem.stop();
    }
}