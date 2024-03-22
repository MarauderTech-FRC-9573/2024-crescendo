package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.IntakeSubsystem;
// IntakeReleaser is for the brush motors to release the note when the note is near the shooter.
/*Basicly the same code from the shootersubsystem with different names.
 *
*/
import frc.robot.subsystems.ShooterSubsystem;

public class IntakeReleaser extends Command {
    IntakeSubsystem intakeSubsystem;
    ShooterSubsystem shooterSubsystem;

    public IntakeReleaser(IntakeSubsystem intakeSubsystem, ShooterSubsystem shooterSubsystem) {
        this.intakeSubsystem = intakeSubsystem;
        this.shooterSubsystem = shooterSubsystem;
        addRequirements(intakeSubsystem);
    }

    @Override
    public void initialize() {
        intakeSubsystem.setIntakeMotor(IntakeConstants.IntakeMotorReleaseSpeed);
        shooterSubsystem.setLaunchWheel(0.3);

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
