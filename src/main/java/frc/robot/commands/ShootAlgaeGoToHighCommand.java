package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.AlgaeSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;

public class ShootAlgaeGoToHighCommand extends Command{

    private AlgaeSubsystem m_algaeSubsystem;
    private ElevatorSubsystem m_elevatorSubsystem;
    private boolean isDone = false;

    private double timer;
    private double startTime;

    public ShootAlgaeGoToHighCommand(ElevatorSubsystem m_elevatorSubsystem, AlgaeSubsystem m_algaeSubsystem) {
        
        this.m_elevatorSubsystem = m_elevatorSubsystem;
        this.m_algaeSubsystem = m_algaeSubsystem;
        isDone = false;

        timer = Timer.getFPGATimestamp();
        startTime = Timer.getFPGATimestamp();
    
    }

    @Override
    public void initialize() {
        m_elevatorSubsystem.setElevatorSetpoint(Constants.ElevatorConstants.Encoder.L4);
        m_algaeSubsystem.setHold();
        isDone = false;
        startTime = Timer.getFPGATimestamp();
        m_elevatorSubsystem.setDelayElevatorForPivot(false);


    }

    @Override
    public void execute() {
        if (m_elevatorSubsystem.elevatorEncoderPosition() > Constants.AlgaeConstants.shootStart) {
            m_algaeSubsystem.setOuttakeSpeed();
            m_elevatorSubsystem.setClearAlgae(true, Constants.ElevatorConstants.Pivot.pivotShoot);
        }

        if (m_elevatorSubsystem.elevatorEncoderPosition() < Constants.AlgaeConstants.shootStart) {
            m_algaeSubsystem.setHold();
            m_elevatorSubsystem.setClearAlgae(false, Constants.ElevatorConstants.Pivot.pivotShoot);
        }

        if (Timer.getFPGATimestamp() - startTime > 1) {
            isDone = true;
        }   
        
    }

    @Override
    public boolean isFinished() {
        return isDone;
    }

    @Override
    public void end(boolean interrupted) {
        m_algaeSubsystem.setHold();
        m_elevatorSubsystem.setElevatorSetpoint(Constants.ElevatorConstants.Encoder.algaeHigh);
        m_elevatorSubsystem.setClearAlgae(false, Constants.ElevatorConstants.Pivot.pivotAlgae);

        m_elevatorSubsystem.setDelayElevatorForPivot(true);


    }

}
