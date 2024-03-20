package frc.robot.commands;

import static frc.robot.Constants.DriveConstants.kD;
import static frc.robot.Constants.DriveConstants.kI;

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
    PIDController pid = new PIDController(IntakeConstants.kPArm, IntakeConstants.kIArm, IntakeConstants.kDArm);
    double setpoint = 0.25;

    public IntakeArmSwitch(IntakeSubsystem intakeSubsystem) {
        this.intakeSubsystem = intakeSubsystem;
        pid.setTolerance(5, 10);
        addRequirements(intakeSubsystem);
    }

    @Override
    public void initialize() {
        //intakeSubsystem.setArmPosition(0.0);
    }

    @Override 
    public void execute() {
        if (intakeSubsystem.getArmPostition() > 0.25) {
            intakeSubsystem.setArmMotor(pid.calculate(intakeSubsystem.getArmPostition(), 0.25));
        } else if (intakeSubsystem.getArmPostition() < 0.25) {
            intakeSubsystem.setArmMotor(pid.calculate(intakeSubsystem.getArmPostition(), 0.25));
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