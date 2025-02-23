package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SmartMotionConfig;
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

    private boolean hasBeenReset = false;
    private boolean isManualMode = false;


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
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Elevator Speed", elevatorSpeed);
        SmartDashboard.putBoolean("Setpoint Mode", setpointMode);
        encoderCurrentPosition = -encoder.getPosition().getValueAsDouble();

        SmartDashboard.putNumber("Elevator Encoder Value", encoderCurrentPosition);
        SmartDashboard.putNumber("Current Elevator Setpoint", defaultSetpoint);

        if (setpointMode) {
            ElevatorPID.setSetpoint(defaultSetpoint);

            elevatorSpeed = MathUtil.clamp(ElevatorPID.calculate(encoderCurrentPosition), 
                                            -Constants.ElevatorConstants.maxElevatorSpeed, 
                                            Constants.ElevatorConstants.maxElevatorSpeed);
        }
        else if (!isManualMode) {
            elevatorSpeed = 0.0;
        }

        if (encoderCurrentPosition >= Constants.ElevatorConstants.Encoder.top) {
            elevatorSpeed = 0;
        }
/* 
        if (isPressed() && !hasBeenReset) {
            elevatorStop();
            zero();
            hasBeenReset = true;
        }
        else if (!isPressed()){
            hasBeenReset = false;
        }*/

        leftClimbMotor.set(elevatorSpeed);
        rightClimbMotor.set(elevatorSpeed);

        SmartDashboard.putBoolean("Left Min Touch Limit Value", isPressed());
    }

    public void elevatorUp(double speed) {
        elevatorSpeed = speed * Constants.ElevatorConstants.maxElevatorSpeed;
        isManualMode = true;
        setpointMode = false;
    }

    public void elevatorDown(double speed) {
        elevatorSpeed = speed * -Constants.ElevatorConstants.maxElevatorSpeed / 2.0; 
        setpointMode = false;
        isManualMode = true;

    }

    public boolean isPressed() {
        return !minLimitTouchLeft.get();
    }

    public void idlePID() {
        //defaultSetpoint = encoderCurrentPosition;
        elevatorSpeed = 0.0;
        setpointMode = false;
    }

    public void elevatorStop() {
        elevatorSpeed = 0.0;
        setpointMode = false;
        isManualMode = false;
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

    public boolean isAtSetpoint(double setpoint) {
        return Math.abs(encoderCurrentPosition - setpoint) 
                < Constants.ElevatorConstants.threshold;
    }

    public double getSpeed() {
        return elevatorSpeed;
    }

    public void initSetPointMode() {
        setpointMode = false;
    }
}