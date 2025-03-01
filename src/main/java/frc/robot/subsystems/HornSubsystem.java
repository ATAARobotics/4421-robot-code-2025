package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class HornSubsystem extends SubsystemBase{
    private SparkMax hornMotor;
    private SparkMaxConfig hornConfig;

    private CANcoder hornEncoder;
    private double hornSpeed = 0.0;

    private double hornPosition;

    private double setpoint;

    private boolean holdSetpoint;

    private PIDController hornPID = new PIDController(
        Constants.HornConstants.kP,
        Constants.HornConstants.kI,
        Constants.HornConstants.kD
    );

    public HornSubsystem(){
        hornMotor = new SparkMax(Constants.HornConstants.hornSparkID, SparkMax.MotorType.kBrushless);
        hornConfig = new SparkMaxConfig();

        hornEncoder = new CANcoder(Constants.HornConstants.hornEncoderID);

        //hornEncoder.setPosition(0);////remove

        hornPosition = -hornEncoder.getPosition().getValueAsDouble();

        hornConfig.inverted(false);
        hornConfig.idleMode(IdleMode.kBrake);
    
        hornMotor.configure(hornConfig, null, null);
        holdSetpoint = false;

        SmartDashboard.putNumber("Horn kP", Constants.HornConstants.kP);
        SmartDashboard.putNumber("Horn kI", Constants.HornConstants.kI);
        SmartDashboard.putNumber("Horn kD", Constants.HornConstants.kD);


    }

    @Override
    public void periodic(){
        hornPID.setPID(SmartDashboard.getNumber("Horn kP", 0.0), SmartDashboard.getNumber("Horn kI", 0.0), SmartDashboard.getNumber("Horn kD", 0.0));
        hornPosition = -hornEncoder.getPosition().getValueAsDouble();
        SmartDashboard.putNumber("Horn Encoder Value", hornPosition);
        //hornSpeed = 0;
        if(holdSetpoint){
        hornPID.setSetpoint(setpoint);

        hornSpeed = MathUtil.clamp(hornPID.calculate(hornPosition), 
                                            -Constants.HornConstants.maxHornSpeed, 
                                            Constants.HornConstants.maxHornSpeed);
        }

        if (hornPosition < 0.05 && hornSpeed < 0){
            hornSpeed = 0.02;
        }
        setHornSpeed(hornSpeed);
        putSmartDashboard();

    }

    public void setHornSpeed(double speed){
        hornMotor.set(speed);
    }

    public void hornRunOut() {
        holdSetpoint = false;
        hornSpeed = 0.2;
    }

    public void hornRunIn() {
        holdSetpoint = false;
        hornSpeed = -0.25;
    }

    public void hornStop() {
        holdSetpoint = true;
        setpoint = hornPosition;
        hornSpeed = 0.0;
    }

    public void putSmartDashboard() {
        SmartDashboard.putNumber("Horn Speed", hornSpeed);
        SmartDashboard.putNumber("Horn Encoder", hornEncoder.getPosition().getValueAsDouble());
    }
}
