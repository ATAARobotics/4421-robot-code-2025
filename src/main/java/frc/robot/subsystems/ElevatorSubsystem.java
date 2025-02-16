package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.generated.TunerConstants;

public class ElevatorSubsystem extends SubsystemBase {
    private double elevatorSpeed = 0;

    public SparkFlex leftClimbMotor = new SparkFlex(Constants.ElevatorConstants.leftClimbMotorID, MotorType.kBrushless);
    public SparkFlex rightClimbMotor = new SparkFlex(Constants.ElevatorConstants.rightClimbMotorID, MotorType.kBrushless); // Put IDs in Constants.java

    public SparkFlexConfig leftConfig;
    public SparkFlexConfig rightConfig;
    
    public boolean setpointMode = false;

    public double defaultSetpoint = Constants.ElevatorConstants.Encoder.L2;
  
    private DigitalInput minLimitTouchLeft;
    private DigitalInput minLimitTouchRight;


    public CANcoder encoder = new CANcoder(Constants.ElevatorConstants.Encoder.encoderID, TunerConstants.kCANBus);
    public double encoderCurrentPosition;

    

    private PIDController ElevatorPID = new PIDController(
        Constants.ElevatorConstants.kP,
        Constants.ElevatorConstants.kI,
        Constants.ElevatorConstants.kD
    );

    public ElevatorSubsystem() {
        leftConfig = new SparkFlexConfig();
        rightConfig = new SparkFlexConfig();

        leftConfig.idleMode(IdleMode.kBrake);
        rightConfig.idleMode(IdleMode.kBrake);

        leftConfig.inverted(true);
        rightConfig.inverted(false);

        leftClimbMotor.configure(leftConfig, null, null);
        rightClimbMotor.configure(rightConfig, null, null);

        minLimitTouchLeft = new DigitalInput(Constants.ElevatorConstants.minLimitTouchLeftPin);
        minLimitTouchRight = new DigitalInput(Constants.ElevatorConstants.minLimitTouchRightPin);
    }

    @Override
    public void periodic() {
        encoderCurrentPosition = -encoder.getPosition().getValueAsDouble();

        SmartDashboard.putNumber("Elevator Encoder Value", encoderCurrentPosition);
        SmartDashboard.putNumber("Current Elevator Setpoint", defaultSetpoint);

        if (setpointMode) {
            ElevatorPID.setSetpoint(defaultSetpoint);

            elevatorSpeed = MathUtil.clamp(ElevatorPID.calculate(encoderCurrentPosition), -Constants.ElevatorConstants.maxElevatorSpeed, Constants.ElevatorConstants.maxElevatorSpeed);
        }

        if (encoderCurrentPosition >= Constants.ElevatorConstants.Encoder.top) {
            elevatorSpeed = 0;
        }

        leftClimbMotor.set(elevatorSpeed);
        rightClimbMotor.set(elevatorSpeed);

        SmartDashboard.putBoolean("Left Min Touch Limit Value", minLimitTouchLeft.get());
        SmartDashboard.putBoolean("Right Min Tough Limit Value", minLimitTouchRight.get());
    }

    public void elevatorUp() {
        elevatorSpeed = Constants.ElevatorConstants.maxElevatorSpeed;
        setpointMode = false;
    }

    public void elevatorDown() {
        elevatorSpeed = -Constants.ElevatorConstants.maxElevatorSpeed / 2.0; 
        setpointMode = false;

    }

    public void elevatorStop() {
        elevatorSpeed = 0.0;
        setpointMode = false;
    }

    public void returnToIntake() {
        setpointMode = true;
        defaultSetpoint = Constants.ElevatorConstants.Encoder.Intake;
    }

    public void setElevatorSetpoint(double setpoint) {
        setpointMode = true;
        defaultSetpoint = setpoint;
    }

    public void zero(){
        encoder.setPosition(0);
    }

    public boolean isSetpointAtL1() {
        return defaultSetpoint == Constants.ElevatorConstants.Encoder.L1;
    }
}