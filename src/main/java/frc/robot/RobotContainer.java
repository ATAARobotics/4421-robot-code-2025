// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.ScoreCoralCommand;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.AbsoluteRotation;
import frc.robot.subsystems.AlignmentSubsystem;
import frc.robot.subsystems.ClimbSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;


public class RobotContainer {
    public SendableChooser<Command> autoChooser;
    private final ClimbSubsystem m_climbSubsystem = new ClimbSubsystem();
    private final ElevatorSubsystem m_elevatorSubsystem = new ElevatorSubsystem();
    private final ShooterSubsystem m_shooterSubsystem = new ShooterSubsystem();
  
    private final double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private final double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    private double speedMultiplier = 1;

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
             // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors

    private final SwerveRequest.FieldCentric driveToSetpoint = new SwerveRequest.FieldCentric() // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for turn motors

    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final CommandSwerveDrivetrain m_Swerve = TunerConstants.createDrivetrain();

    private final CommandXboxController joystick = new CommandXboxController(0);
    private final CommandXboxController operatorJoystick = new CommandXboxController(1);

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    private final AlignmentSubsystem m_alignmentSubsystem = new AlignmentSubsystem(drivetrain);

    private AbsoluteRotation m_AbsoluteRotation = new AbsoluteRotation(() -> joystick.getRightX(), () -> joystick.getRightY(), () -> drivetrain.getState().Pose.getRotation().getRadians());

    private boolean scoring = false;
    private boolean isAbsoluteHeading = false;

    private Command coralL1Command;
    private Command coralL2Command;
    private Command coralL3Command;
    private Command coralL4Command;

    private Command intake;

    public RobotContainer() {
        scoring = false;
        isAbsoluteHeading = false;
        speedMultiplier = 1;

        coralL1Command = new ScoreCoralCommand(m_shooterSubsystem, m_elevatorSubsystem, 1);
        coralL2Command = new ScoreCoralCommand(m_shooterSubsystem, m_elevatorSubsystem, 2);
        coralL3Command = new ScoreCoralCommand(m_shooterSubsystem, m_elevatorSubsystem, 3);
        coralL4Command = new ScoreCoralCommand(m_shooterSubsystem, m_elevatorSubsystem, 4);
        intake = new IntakeCommand(m_shooterSubsystem, m_elevatorSubsystem);
        
        NamedCommands.registerCommand("Intake", intake);
        NamedCommands.registerCommand("ScoreCoralL1", coralL1Command);
        NamedCommands.registerCommand("ScoreCoralL2", coralL2Command);
        NamedCommands.registerCommand("ScoreCoralL3", coralL3Command);
        NamedCommands.registerCommand("ScoreCoralL4", coralL4Command);

        autoChooser = AutoBuilder.buildAutoChooser();
        SmartDashboard.putData("Auto Chooser", autoChooser);

        

        
        configureBindings();
    }

    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            // if right trigger is being held down, scoring will be true, thus alignment will be active
            drivetrain.applyRequest(() ->
                scoring ? driveToSetpoint.withVelocityX(m_alignmentSubsystem.getOutputs()[0])
                .withVelocityY(m_alignmentSubsystem.getOutputs()[1])
                .withRotationalRate(m_alignmentSubsystem.getOutputs()[2]) :
                drive.withVelocityX(-joystick.getLeftY() * MaxSpeed * speedMultiplier) // Drive forward with negative Y (forward)
                    .withVelocityY(-joystick.getLeftX() * MaxSpeed * speedMultiplier) // Drive left with negative X (left)
                    .withRotationalRate(isAbsoluteHeading ? speedMultiplier * m_AbsoluteRotation.rotationSpeed() : -joystick.getRightX() * MaxAngularRate * speedMultiplier)
                    .withRotationalDeadband(isAbsoluteHeading ? 0.0 : speedMultiplier * MaxAngularRate * Constants.SwerveConstants.angularDeadBand) // Drive counterclockwise with negative X (left)
                    .withDeadband(speedMultiplier * MaxSpeed * Constants.SwerveConstants.linearDeadBand)
            )
        );

        // toggle absolute heading mode on B press on the second controller
        operatorJoystick.b().onTrue(
            new InstantCommand(() -> isAbsoluteHeading = !isAbsoluteHeading)
        );

        operatorJoystick.y().onTrue(new InstantCommand(() -> speedMultiplier *= 0.1))
                            .onFalse(new InstantCommand(() -> speedMultiplier *= 10)); // slowmode?
        // joystick.a().whileTrue(drivetrain.applyRequest(() -> brake));
        operatorJoystick.povRight().whileTrue(drivetrain.applyRequest(() ->
            point.withModuleDirection(new Rotation2d(0.0))
        ));

        operatorJoystick.povDown().onTrue(coralL2Command);

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        
        joystick.back().and(joystick.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        joystick.back().and(joystick.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        joystick.start().and(joystick.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        joystick.start().and(joystick.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        // reset the field-centric heading on left bumper press
        joystick.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));


        drivetrain.registerTelemetry(logger::telemeterize);
      
      
        joystick.y().onTrue(new InstantCommand(m_climbSubsystem::climbUp)).onFalse(new InstantCommand(m_climbSubsystem::stop));
        joystick.a().onTrue(new InstantCommand(m_climbSubsystem::climbDown)).onFalse(new InstantCommand(m_climbSubsystem::stop));
        operatorJoystick.a().onTrue(new InstantCommand(() -> {m_elevatorSubsystem.zero(); m_elevatorSubsystem.elevatorStop();}));
        joystick.leftTrigger(0.1).whileTrue(new RunCommand(() -> m_elevatorSubsystem.elevatorDown(joystick.getLeftTriggerAxis()))).onFalse(new InstantCommand(() -> m_elevatorSubsystem.elevatorStop()));
        joystick.rightTrigger(0.1).whileTrue(new RunCommand(() -> m_elevatorSubsystem.elevatorUp(joystick.getRightTriggerAxis()))).onFalse(new InstantCommand(() -> m_elevatorSubsystem.elevatorStop()));



        // Rest
        joystick.povDown().onTrue(new InstantCommand(
            () -> m_elevatorSubsystem.setElevatorSetpoint(Constants.ElevatorConstants.Encoder.L1)
        ));

        // L2
        joystick.povUp().onTrue(new InstantCommand(
            () -> m_elevatorSubsystem.setElevatorSetpoint(Constants.ElevatorConstants.Encoder.L2)
        ));

        joystick.povLeft().onTrue(new InstantCommand(
            () -> m_elevatorSubsystem.setElevatorSetpoint(Constants.ElevatorConstants.Encoder.L3)
        ));

        joystick.povRight().onTrue(new InstantCommand(
            () -> m_elevatorSubsystem.setElevatorSetpoint(Constants.ElevatorConstants.Encoder.L4)
        ));

        joystick.button(10).onTrue(new InstantCommand(
            () -> m_elevatorSubsystem.setElevatorSetpoint(Constants.ElevatorConstants.Encoder.Intake)
        ));

        operatorJoystick.x().onTrue(new InstantCommand(
            () -> m_elevatorSubsystem.setElevatorSetpoint(Constants.ElevatorConstants.Encoder.rest)
        ));

        operatorJoystick.rightBumper().onTrue(new InstantCommand(
            () -> m_elevatorSubsystem.elevatorStop()
        ));
        
        operatorJoystick.leftBumper().onTrue(new InstantCommand(() -> m_Swerve.zeroGyro()));
       

        // bool to acitvate alignment, press both the up buttom on the d-pad and the a button on second controller
        operatorJoystick.povUp().onTrue(new InstantCommand(() -> { m_alignmentSubsystem.updateGoalPose(); scoring = true;})).onFalse(new InstantCommand(() -> scoring = false));
        
        joystick.rightBumper()
            .onTrue(m_elevatorSubsystem.isAtSetpoint(Constants.ElevatorConstants.Encoder.L1) ? new InstantCommand(() -> m_shooterSubsystem.shootL1()) : new InstantCommand(() -> m_shooterSubsystem.shoot()))
            .onFalse(new InstantCommand(() -> m_shooterSubsystem.stop()));

        joystick.b()
            .onTrue(new InstantCommand(m_shooterSubsystem::toggleOverride));

    }

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
      }
      public void resetClimbAndElevator() {
        m_climbSubsystem.initHoldMode();
        m_elevatorSubsystem.initSetPointMode();;
      }
}
