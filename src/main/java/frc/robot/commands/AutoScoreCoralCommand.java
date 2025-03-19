package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.AlignmentSubsystem;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class AutoScoreCoralCommand extends Command{

    private CommandSwerveDrivetrain drivetrain;
    private ElevatorSubsystem elevator;
    private ShooterSubsystem shooter;
    private AlignmentSubsystem align;

    private final SwerveRequest.FieldCentric driveToSetpoint = new SwerveRequest.FieldCentric()
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    private boolean startedScoring;
    private boolean isDone;

    private double desiredPosition;
    
    public AutoScoreCoralCommand(CommandSwerveDrivetrain drivetrain, 
                                 ElevatorSubsystem elevator,
                                 ShooterSubsystem shooter,
                                 AlignmentSubsystem align) {
        this.drivetrain = drivetrain;
        this.elevator = elevator;
        this.shooter = shooter;
        this.align = align;
    }

    @Override
    public void initialize() {
        startedScoring = false;
        isDone = false;
        
        desiredPosition = Constants.ElevatorConstants.Encoder.L4;
        
        elevator.setElevatorSetpoint(desiredPosition);
        shooter.stop();

        drivetrain.zeroGyro();

        System.out.println("Started Elevator Score Coral Command *********************##################");
    }

    @Override
    public void execute() {
        // drivetrain.applyRequest(() ->
        // driveToSetpoint.withVelocityX(align.getOutputs()[0])
        //         .withVelocityY(align.getOutputs()[1])
        //         .withRotationalRate(align.getOutputs()[2]));

        PPHolonomicDriveController.overrideXYFeedback(() -> {System.out.println("getting x " + align.getOutputs()[0]); return -align.getOutputs()[0];}, () -> -align.getOutputs()[1]);
        // PPHolonomicDriveController.overrideXFeedback(() -> {System.out.println("CALLING X OVERRIDE XXXXXXXXXXX-----XXXXXXXXXXX");return align.getOutputs()[0];});
        // PPHolonomicDriveController.overrideYFeedback(() -> {System.out.println("CALLING Y OVERRIDE YYYYYYYYYYY-----YYYYYYYYYYY");return align.getOutputs()[1];});
        PPHolonomicDriveController.overrideRotationFeedback(() -> {return align.getOutputs()[2];});



        if (!startedScoring && align.aligned() && elevator.isAtSetpoint(Constants.ElevatorConstants.Encoder.L4)) {
            shooter.shoot();
            startedScoring = true;
        }
        else if(startedScoring && shooter.getState() == ShooterSubsystem.ShooterState.IDLE) {
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
        PPHolonomicDriveController.clearFeedbackOverrides();
    }
}
