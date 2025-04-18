package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.AlgaeSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;

public class AlgaeIntakeHighCommand extends Command {
    // this is the command that puts the elevator in cleaning height and runs pivot outward and runs intake
    private final AlgaeSubsystem algaeSubsystem;
    private final ElevatorSubsystem elevatorSubsystem;

    private boolean isReady = false;

    public AlgaeIntakeHighCommand(AlgaeSubsystem algaeSubsystem, ElevatorSubsystem elevatorSubsystem) {
        this.algaeSubsystem = algaeSubsystem;
        this.elevatorSubsystem = elevatorSubsystem;
    }

    @Override
    public void initialize() {
        elevatorSubsystem.setElevatorSetpoint(Constants.ElevatorConstants.Encoder.algaeHigh);
        elevatorSubsystem.setClearAlgae(true, Constants.ElevatorConstants.Pivot.pivotAlgae);
        algaeSubsystem.setIntakeSpeed();
        isReady = false;
    }

    @Override
    public void execute() {
        elevatorSubsystem.setClearAlgae(true, Constants.ElevatorConstants.Pivot.pivotAlgae);
        if (elevatorSubsystem.isAtSetpoint(Constants.ElevatorConstants.Encoder.algaeHigh) && Math.abs(elevatorSubsystem.getPivotEncoderPosition() - Constants.ElevatorConstants.Pivot.pivotAlgae) < 0.04) {
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
