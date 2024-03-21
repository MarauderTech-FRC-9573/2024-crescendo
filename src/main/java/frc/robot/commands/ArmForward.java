package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.IntakeSubsystem;
// IntakeReceiver is for the brush motors to receive the note from the ground.
/*Basicly the same code from the shootersubsystem with different names.
 *
*/

public class ArmForward extends Command {
    IntakeSubsystem intakeSubsystem;
    public ArmForward(IntakeSubsystem intakeSubsystem) {
        this.intakeSubsystem = intakeSubsystem;
        addRequirements(intakeSubsystem);
    }

    @Override
    public void initialize() {
        intakeSubsystem.setArmMotor(IntakeConstants.ArmMotorMoveForwardSpeed);
    }

    @Override 
    public void execute() {
        
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