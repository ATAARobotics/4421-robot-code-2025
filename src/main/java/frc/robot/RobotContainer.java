// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.AlignmentSubsystem;
import frc.robot.subsystems.ClimbSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import edu.wpi.first.wpilibj2.command.InstantCommand;

public class RobotContainer {
    private final ClimbSubsystem m_climbSubsystem = new ClimbSubsystem();
    private final ElevatorSubsystem m_elevatorSubsystem = new ElevatorSubsystem();
    private final ShooterSubsystem m_shooterSubsystem = new ShooterSubsystem();
  
    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.2) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors

    private final SwerveRequest.FieldCentric driveToSetpoint = new SwerveRequest.FieldCentric() // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for turn motors

    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final CommandXboxController joystick = new CommandXboxController(0);
    private final CommandXboxController operatorJoystick = new CommandXboxController(1);

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    private final AlignmentSubsystem m_alignmentSubsystem = new AlignmentSubsystem(drivetrain);

    private boolean scoring = false;

    public RobotContainer() {
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
                drive.withVelocityX(-joystick.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(-joystick.getLeftX() * MaxSpeed) // Drive left with negative X (left)
                    .withRotationalRate(-joystick.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
            )
        );

        // joystick.a().whileTrue(drivetrain.applyRequest(() -> brake));
        operatorJoystick.povRight().whileTrue(drivetrain.applyRequest(() ->
            point.withModuleDirection(new Rotation2d(0.0))
        ));

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
        // joystick.a().onTrue(new InstantCommand(m_elevatorSubsystem::zero));
        joystick.x().onTrue(new InstantCommand(m_elevatorSubsystem::elevatorUp)).onFalse(new InstantCommand(m_elevatorSubsystem::elevatorStop));
        joystick.b().onTrue(new InstantCommand(m_elevatorSubsystem::elevatorDown)).onFalse(new InstantCommand(m_elevatorSubsystem::elevatorStop));

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
       

        // bool to acitvate alignment
        // joystick.rightTrigger().onTrue(new InstantCommand(() -> scoring = true)).onFalse(new InstantCommand(() -> scoring = false));
        
        joystick.rightBumper()
            .onTrue(m_elevatorSubsystem.isSetpointAtL1() ? new InstantCommand(() -> m_shooterSubsystem.shootL1()) : new InstantCommand(() -> m_shooterSubsystem.shoot()))
            .onFalse(new InstantCommand(() -> m_shooterSubsystem.stop()));

    }

    public Command getAutonomousCommand() {
        return Commands.print("No autonomous command configured");
    }
}
