package frc.robot.commands;

import java.util.function.IntSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class ScoreCoralCommand extends Command{
    
    private ShooterSubsystem shooter;
    private ElevatorSubsystem elevator;

    private int position;
    private double desiredPosition;

    private boolean startedScoring;
    private boolean doneScoring;
    private boolean isDone;

    public ScoreCoralCommand(ShooterSubsystem shooter, ElevatorSubsystem elevator) {
        this.shooter = shooter;
        this.elevator = elevator;

        startedScoring = false;
        doneScoring = false;
        isDone = false;
    }

    @Override
    public void initialize() {
        startedScoring = false;
        doneScoring = false;
        isDone = false;
        
        desiredPosition = Constants.ElevatorConstants.Encoder.L4;
        
        elevator.setElevatorSetpoint(desiredPosition);
        shooter.stop();

        System.out.println("Started Elevator Score Coral Command *********************##################");
    }
    
    @Override
    public void execute() {
        
        if(!startedScoring && elevator.isAtSetpoint(desiredPosition)) {
            System.out.println("at setpoint");
            startedScoring = true;
            shooter.shoot();
        }
        if(startedScoring && shooter.getState() == ShooterSubsystem.ShooterState.IDLE) {
            shooter.stop();
            isDone = true;
        } 
    }

    @Override
    public boolean isFinished() {
        return isDone;
    }

    @Override
    public void end(boolean isInterrupted) {
        elevator.returnToIntake();
    }
}
