package frc.robot.commands;

import java.util.Optional;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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

    private double xSpeed = 0;
    private double ySpeed = 0;
    private double rotSpeed = 0;

    
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
        align.setBiasedSideFalse();
        hasSetDesiredPosition = false;
        initialTime = Timer.getFPGATimestamp();
        elapsedTime = Timer.getFPGATimestamp() - initialTime;
        startedScoring = false;
        isDone = false;
        
        desiredPosition = Constants.ElevatorConstants.Encoder.L4;
        
        shooter.stop();

        drivetrain.checkFirstZeroGyro();

        xSpeed = align.getOutputs()[0];
        ySpeed = align.getOutputs()[1];
        rotSpeed = align.getOutputs()[2];

        System.out.println("Started Elevator Score Coral Command *********************##################");
    }

    @Override
    public void execute() {
        
        // negative xy for red side, positive all for blue
        

        Optional<DriverStation.Alliance> alliance = DriverStation.getAlliance();
        if (alliance.isPresent()) {
            xSpeed = (alliance.get() == DriverStation.Alliance.Red) ? -align.getOutputs()[0] : align.getOutputs()[0];
        ySpeed = (alliance.get() == DriverStation.Alliance.Red) ? -align.getOutputs()[1] : align.getOutputs()[1];
        rotSpeed = align.getOutputs()[2];
        } else{
            // updateGoalPose();
            xSpeed = -align.getOutputs()[0];
        ySpeed = -align.getOutputs()[1];
        rotSpeed = align.getOutputs()[2];
        }
        
        elapsedTime = Timer.getFPGATimestamp() - initialTime;

        if (!hasSetDesiredPosition && elapsedTime > 0.07) {
            elevator.setElevatorSetpoint(desiredPosition);
            hasSetDesiredPosition = true;
        }

        // drivetrain.applyRequest(() ->
        // driveToSetpoint.withVelocityX(align.getOutputs()[0])
        //         .withVelocityY(align.getOutputs()[1])
        //         .withRotationalRate(align.getOutputs()[2]));

        if (Math.abs(xSpeed) > 0.01 || Math.abs(ySpeed) > 0.07 || Math.abs(rotSpeed) > 0.07 * Math.PI) {
        PPHolonomicDriveController.overrideXYFeedback(() -> {return xSpeed;}, () -> {return ySpeed;});
        PPHolonomicDriveController.overrideRotationFeedback(() -> {return rotSpeed;});

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
        align.setBiasedSideTrue();
        shooter.setOverrideTrue();
        elevator.delayedReturnTOIntake();
        // elevator.returnToIntake();
        PPHolonomicDriveController.clearFeedbackOverrides();
    }
}
