package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.IntakeSubsystem;
// IntakeReceiver is for the brush motors to receive the note from the ground.
/*Basicly the same code from the shootersubsystem with different names.
 *
*/

public class IntakeArmSwitch extends Command {
    IntakeSubsystem intakeSubsystem;

    public IntakeArmSwitch(IntakeSubsystem intakeSubsystem) {
        this.intakeSubsystem = intakeSubsystem;
        addRequirements(intakeSubsystem);
    }

    @Override
    public void initialize() {
        //intakeSubsystem.setArmPosition(0.0);
    }

    @Override 
    public void execute() {
        if (intakeSubsystem.getArmPostition() > 0.25) {
            intakeSubsystem.setArmMotor(IntakeConstants.ArmMotorMoveForwardSpeed);
        }
        else if (intakeSubsystem.getArmPostition() < 0.25) {
            intakeSubsystem.setArmMotor(IntakeConstants.ArmMotorMoveBackwardSpeed);
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