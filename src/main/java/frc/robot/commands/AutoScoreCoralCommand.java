package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.wpilibj.Timer;
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

    private double initialTime;
    private double elapsedTime;
    private double timeSinceScoring;

    private boolean hasSetDesiredPosition;

    
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
        hasSetDesiredPosition = false;
        initialTime = Timer.getFPGATimestamp();
        elapsedTime = Timer.getFPGATimestamp() - initialTime;
        startedScoring = false;
        isDone = false;
        
        desiredPosition = Constants.ElevatorConstants.Encoder.L4;
        
        shooter.stop();

        drivetrain.checkFirstZeroGyro();

        System.out.println("Started Elevator Score Coral Command *********************##################");
    }

    @Override
    public void execute() {
        elapsedTime = Timer.getFPGATimestamp() - initialTime;

        if (!hasSetDesiredPosition && elapsedTime > 0.07) {
            elevator.setElevatorSetpoint(desiredPosition);
            hasSetDesiredPosition = true;
        }

        // drivetrain.applyRequest(() ->
        // driveToSetpoint.withVelocityX(align.getOutputs()[0])
        //         .withVelocityY(align.getOutputs()[1])
        //         .withRotationalRate(align.getOutputs()[2]));

        if (Math.abs(align.getOutputs()[0]) > 0.01 || Math.abs(align.getOutputs()[1]) > 0.07 || Math.abs(align.getOutputs()[2]) > 0.07 * Math.PI) {
        PPHolonomicDriveController.overrideXYFeedback(() -> {System.out.println("getting x " + align.getOutputs()[0]); return -align.getOutputs()[0];}, () -> {return -align.getOutputs()[1];});
        PPHolonomicDriveController.overrideRotationFeedback(() -> {return align.getOutputs()[2];});

        // PPHolonomicDriveController.overrideXYFeedback(() -> {System.out.println("$#^#%^#%^#%^#%^#%^%#%^#%^#%^#%^#%^"); return 0.0;}, () -> {return 0.0;});
        // PPHolonomicDriveController.overrideRotationFeedback(() -> {return 0.0;});
        }
        else {
            PPHolonomicDriveController.overrideXYFeedback(() -> {return 0.0;}, () -> {return 0.0;});
        PPHolonomicDriveController.overrideRotationFeedback(() -> {return 0.0;});
        }

        if (!startedScoring && align.aligned() && elevator.isAtSetpoint(Constants.ElevatorConstants.Encoder.L4) || 
            !startedScoring && elapsedTime > Constants.SwerveConstants.autoAlignTimeConstraint && elevator.isAtSetpoint(Constants.ElevatorConstants.Encoder.L4)) {
            shooter.shoot();
            startedScoring = true;
            timeSinceScoring = Timer.getFPGATimestamp();
        }
        else if(startedScoring && Timer.getFPGATimestamp() - timeSinceScoring > Constants.SwerveConstants.shootingCommandWaitCommand) {
            isDone = true;
        }
    }

    @Override
    public boolean isFinished() {
        return isDone;
    }

    @Override
    public void end(boolean isInterrupted) {
        shooter.setOverrideTrue();
        elevator.delayedReturnTOIntake();
        //elevator.returnToIntake();
        PPHolonomicDriveController.clearFeedbackOverrides();
    }
}
