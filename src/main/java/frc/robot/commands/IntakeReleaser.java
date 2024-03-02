package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.IntakeSubsystem;
// IntakeReleaser is for the brush motors to release the note when the note is near the shooter.
/*Basicly the same code from the shootersubsystem with different names.
 *
*/

public class IntakeReleaser extends Command {
    IntakeSubsystem IntakeSubsystem;

    public IntakeReleaser(IntakeSubsystem IntakeSubsystem) {
        this.IntakeSubsystem = intakeSubsystem;
        addRequirements(intakeSubsystem);
    }

    @Override
    public void initialize() {
        intakeSubsystem.setBrushWheel(IntakeConstants.BrushMotorRelease);

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
