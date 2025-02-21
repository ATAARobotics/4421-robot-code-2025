package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSubsystem;

public class IntakeCommand extends Command {

    private ShooterSubsystem shooter;

    boolean isDone;
    
    public IntakeCommand(ShooterSubsystem shooter) {
        this.shooter = shooter;

        isDone = false;
    }

    @Override
    public void initialize() {
        isDone = false;
        shooter.intake();
    }

    @Override
    public void execute() {
        if(shooter.getState() == ShooterSubsystem.ShooterState.IDLE) {
            isDone = true;
        }
    }

    @Override
    public boolean isFinished() {
        return isDone;
    }
}
