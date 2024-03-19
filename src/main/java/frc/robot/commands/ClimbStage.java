package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ClimberSubsystem;

public class ClimbStage extends Command {
    private ClimberSubsystem climberSubsystem;
    private double power;
    private boolean isExtending;

    public ClimbStage(ClimberSubsystem climberSubsystem, double power, boolean isExtending) {
        this.climberSubsystem = climberSubsystem;
        this.power = power;
        this.isExtending = isExtending;
        addRequirements(climberSubsystem);
    }

    @Override
    public void initialize() {
        if (isExtending) {
            climberSubsystem.extend();
        } else {
            climberSubsystem.retract();
        }
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
        climberSubsystem.stop();
    }
}