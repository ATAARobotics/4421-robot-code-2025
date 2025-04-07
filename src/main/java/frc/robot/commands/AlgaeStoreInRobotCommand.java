package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.AlgaeSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;

// nicely tucks it in
public class AlgaeStoreInRobotCommand extends Command {
    private final AlgaeSubsystem algaeSubsystem;
    private final ElevatorSubsystem elevatorSubsystem;

    private boolean isReady = false;

    public AlgaeStoreInRobotCommand(AlgaeSubsystem algaeSubsystem, ElevatorSubsystem elevatorSubsystem) {
        this.algaeSubsystem = algaeSubsystem;
        this.elevatorSubsystem = elevatorSubsystem;
    }

    @Override
    public void initialize() {
        elevatorSubsystem.setElevatorSetpoint(Constants.ElevatorConstants.Encoder.Intake);
        elevatorSubsystem.setClearAlgae(false, Constants.ElevatorConstants.Pivot.pivotIntake);
        algaeSubsystem.setIntakeSpeed();
        isReady = false;
    }

    @Override
    public void execute() {
        if (elevatorSubsystem.isAtSetpoint(Constants.ElevatorConstants.Encoder.Intake) && Math.abs(elevatorSubsystem.getPivotEncoderPosition() - Constants.ElevatorConstants.Pivot.pivotIntake) < 0.02) {
            isReady = true;
        }
    }

    @Override
    public boolean isFinished() {
        return isReady;
    }

    @Override
    public void end(boolean interrupted) {
        algaeSubsystem.setHold();
    }

}


