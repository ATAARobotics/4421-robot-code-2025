package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class IntakeCommand extends Command {

    private ShooterSubsystem shooter;
    private ElevatorSubsystem elevatorSubsystem;

    boolean isDone;
    
    public IntakeCommand(ShooterSubsystem shooter, ElevatorSubsystem elevatorSubsystem) {
        this.shooter = shooter;
        this.elevatorSubsystem = elevatorSubsystem;

        isDone = false;
    }

    @Override
    public void initialize() {
       
        System.out.println("Started Intake Command *********************##################");

        isDone = false;
        shooter.stop();
        shooter.setOverrideTrue();
        elevatorSubsystem.setElevatorSetpoint(Constants.ElevatorConstants.Encoder.Intake);
    }

    @Override
    public void execute() {
        SmartDashboard.putBoolean("isDone", isDone);
        if(shooter.checkLaserCan()) {
            isDone = true;
        }
    }

    @Override
    public boolean isFinished() {
        shooter.setOverrideFalse();
        return isDone;
    }
}

