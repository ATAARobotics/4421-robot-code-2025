package frc.robot.subsystems;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SmartMotionConfig;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.generated.TunerConstants;

public class ElevatorSubsystem extends SubsystemBase {
    private double elevatorSpeed = 0;

    public SparkMax pivotMotor = new SparkMax(Constants.ElevatorConstants.Pivot.pivotSparkID, MotorType.kBrushless);

    public SparkFlex leftClimbMotor = new SparkFlex(Constants.ElevatorConstants.leftClimbMotorID, MotorType.kBrushless);
    public SparkFlex rightClimbMotor = new SparkFlex(Constants.ElevatorConstants.rightClimbMotorID, MotorType.kBrushless); // Put IDs in Constants.java

    public SparkFlexConfig leftConfig;
    public SparkFlexConfig rightConfig;

    public SparkMaxConfig pivotConfig;
    
    public boolean setpointMode = false;

    public double defaultSetpoint = Constants.ElevatorConstants.Encoder.L2;
  
    private DigitalInput minLimitTouchLeft;

    private boolean hasBeenReset = false;
    private boolean isManualMode = false;

    private boolean prevPressed = false;

    public CANcoder encoder = new CANcoder(Constants.ElevatorConstants.Encoder.encoderID, new CANBus("rio"));
    public CANcoder pivotEncoder = new CANcoder(Constants.ElevatorConstants.Pivot.pivotEncoderID, new CANBus("rio"));

    public double encoderCurrentPosition;
    public double pivotEncoderPosition;

    public double pivotSpeed = 0.0;

    public boolean hasBeenZero = false;

    // public double feedForward = Constants.ElevatorConstants.feedForward;

    private boolean justDoneAutoCommand;
    private double timeAtDone = 0;

    private boolean clearAlgae;

    private PIDController pivotPID = new PIDController(
        Constants.ElevatorConstants.Pivot.pivotkP,
        Constants.ElevatorConstants.Pivot.pivotkI,
        Constants.ElevatorConstants.Pivot.pivotkD
    );

    private PIDController elevatorPID = new PIDController(
        Constants.ElevatorConstants.kP,
        Constants.ElevatorConstants.kI,
        Constants.ElevatorConstants.kD
    );


    public ElevatorSubsystem() {
        justDoneAutoCommand = false;
        hasBeenZero = false;

        clearAlgae = false;

        leftConfig = new SparkFlexConfig();
        rightConfig = new SparkFlexConfig();
        pivotConfig = new SparkMaxConfig();

        leftConfig.idleMode(IdleMode.kBrake);
        rightConfig.idleMode(IdleMode.kBrake);
        pivotConfig.idleMode(IdleMode.kBrake);


        leftConfig.inverted(true);
        rightConfig.inverted(false);
        pivotConfig.inverted(false);

        leftConfig.smartCurrentLimit(40);
        rightConfig.smartCurrentLimit(40);
        pivotConfig.smartCurrentLimit(20);

        leftClimbMotor.configure(leftConfig, null, null);
        rightClimbMotor.configure(rightConfig, null, null);
        pivotMotor.configure(pivotConfig, null, null);


        //SmartDashboard.putNumber("Elevator P", defaultSetpoint)
        prevPressed = false;

        // SmartDashboard.putNumber("Feed Forward", Constants.ElevatorConstants.feedForward);
    }

    @Override
    public void periodic() {

        pivotPID.setPID(Constants.ElevatorConstants.Pivot.pivotkP, Constants.ElevatorConstants.Pivot.pivotkI, Constants.ElevatorConstants.Pivot.pivotkD);
        elevatorPID.setPID(Constants.ElevatorConstants.kP, Constants.ElevatorConstants.kI, Constants.ElevatorConstants.kD);

        encoderCurrentPosition = encoder.getPosition().getValueAsDouble();
        pivotEncoderPosition = pivotEncoder.getAbsolutePosition().getValueAsDouble();

        SmartDashboard.putNumber("Elevator Encoder Value", encoderCurrentPosition);


        // if (isPressed() && !prevPressed) {
        //     prevPressed = true;
        //     zero();
        // }
        // if(!isPressed()) {
        //     prevPressed = false;
        // }


        if(justDoneAutoCommand && Timer.getFPGATimestamp() - timeAtDone > 0.2) {
            justDoneAutoCommand = false;
            this.returnToIntake();
        }

        if (setpointMode) {
            elevatorPID.setSetpoint(defaultSetpoint);

            elevatorSpeed = MathUtil.clamp(elevatorPID.calculate(encoderCurrentPosition), 
                                            -Constants.ElevatorConstants.maxElevatorSpeed * 0.66, 
                                            Constants.ElevatorConstants.maxElevatorSpeed);
        }

        else if (!isManualMode) {
            elevatorSpeed = 0.0;
        }

        if (encoderCurrentPosition >= Constants.ElevatorConstants.Encoder.top) {
            elevatorSpeed = -0.02;
        }

        controlPivot();

        // if (elevatorSpeed < -0.2 && pivotEncoderPosition > 0.02) {
        //     elevatorSpeed = -0.15;
        // }
/* 
        if (isPressed() && !hasBeenReset) {
            elevatorStop();
            zero();
            hasBeenReset = true;
        }
        else if (!isPressed()){
            hasBeenReset = false;
        }*/

        if ((!SmartDashboard.getBoolean("Check Shooter LaserCAN", false) && SmartDashboard.getBoolean("Check LaserCAN", false)) && elevatorSpeed > 0.2) {
            elevatorSpeed = 0;
        }

        leftClimbMotor.set(elevatorSpeed);
        rightClimbMotor.set(elevatorSpeed);

        pivotMotor.set(pivotSpeed);

    }

    public void setClearAlgae(boolean value) {
        clearAlgae = value;
    }

    private void controlPivot() {
        if (clearAlgae) {
            pivotPID.setSetpoint(Constants.ElevatorConstants.Pivot.pivotAlgae);
        }
        else if(encoderCurrentPosition >= Constants.ElevatorConstants.Encoder.pivotL4Point){
            pivotPID.setSetpoint(Constants.ElevatorConstants.Pivot.pivotL4);
        }
        else if (encoderCurrentPosition <= Constants.ElevatorConstants.Encoder.pivotInBetweenPoint) {
            pivotPID.setSetpoint(Constants.ElevatorConstants.Pivot.pivotIntake);
        }
        else if (encoderCurrentPosition >= Constants.ElevatorConstants.Encoder.pivotInBetweenPoint && encoderCurrentPosition <= Constants.ElevatorConstants.Encoder.pivotL4Point) {
            pivotPID.setSetpoint(Constants.ElevatorConstants.Pivot.pivotInBetween);
        } 

        pivotSpeed = MathUtil.clamp(pivotPID.calculate(pivotEncoderPosition), 
                                        -Constants.ElevatorConstants.Pivot.pivotMaxSpeed, 
                                        Constants.ElevatorConstants.Pivot.pivotMaxSpeed);
    }

    public void elevatorUp(double speed) {
        elevatorSpeed = speed * Constants.ElevatorConstants.maxElevatorSpeed;
        isManualMode = true;
        setpointMode = false;
    }

    public void elevatorDown(double speed) {
        elevatorSpeed = speed * -Constants.ElevatorConstants.maxElevatorSpeed; 
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

    public void delayedReturnTOIntake() {
        justDoneAutoCommand = true;
        timeAtDone = Timer.getFPGATimestamp();
    }

    public void setElevatorSetpoint(double setpoint) {
        setpointMode = true;
        defaultSetpoint = setpoint;
    }

    public void zero(){  
        if (!hasBeenZero || Math.abs(encoderCurrentPosition) > 0.15) {
            encoder.setPosition(0);
            setpointMode = false;
            System.out.println("Zero Elevator");
        }
    }

    public boolean isAtSetpoint(double setpoint) {
        return Math.abs(encoderCurrentPosition - setpoint) 
                < Constants.ElevatorConstants.threshold;
    }

    public boolean isAtL1()
 {
    return Math.abs(encoderCurrentPosition - Constants.ElevatorConstants.Encoder.L1) < 0.1;
 }
    public double getSpeed() {
        return elevatorSpeed;
    }

    public void initSetPointMode() {
        setpointMode = false;
    }

    public void runPivot() {
        pivotSpeed = 0.1;
    }

    public void reversePivot() {
        pivotSpeed = -0.1;
    }

    public void stopPivot() {
        pivotSpeed = 0.0;
    }
}