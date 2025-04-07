package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.AlgaeSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;

public class AlgaeIntakeCommand extends Command {
    // this is the command that puts the elevator in cleaning height and runs pivot outward and runs intake
    private final AlgaeSubsystem algaeSubsystem;
    private final ElevatorSubsystem elevatorSubsystem;

    private boolean isReady = false;

    public AlgaeIntakeCommand(AlgaeSubsystem algaeSubsystem, ElevatorSubsystem elevatorSubsystem) {
        this.algaeSubsystem = algaeSubsystem;
        this.elevatorSubsystem = elevatorSubsystem;
    }

    @Override
    public void initialize() {
        elevatorSubsystem.setElevatorSetpoint(Constants.ElevatorConstants.Encoder.algaeLow);
        elevatorSubsystem.setClearAlgae(true, Constants.ElevatorConstants.Pivot.pivotAlgae);
        algaeSubsystem.setIntakeSpeed();
        isReady = false;
    }

    @Override
    public void execute() {
        elevatorSubsystem.setClearAlgae(true, Constants.ElevatorConstants.Pivot.pivotAlgae);
        if (elevatorSubsystem.isAtSetpoint(Constants.ElevatorConstants.Encoder.algaeLow) && Math.abs(elevatorSubsystem.getPivotEncoderPosition() - Constants.ElevatorConstants.Pivot.pivotAlgae) < 0.02) {
            isReady = true;
        }
    }

    @Override
    public boolean isFinished() {
        return isReady;
    }

    @Override
    public void end(boolean interrupted) {
        algaeSubsystem.setIntakeSpeed();
    }

}
