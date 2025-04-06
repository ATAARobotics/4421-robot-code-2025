package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class AlgaeSubsystem extends SubsystemBase{
    
    private SparkMax algaeMotor;
    private CANcoder algaeEncoder;

    private double algaeSpeed;
    private boolean hold = false;
    private double algaeEncoderPosition;
    private double algaeSetpoint;

    private boolean hasReset = false;

    private PIDController algaePID = new PIDController(
        Constants.AlgaeConstants.kP,
        Constants.AlgaeConstants.kI,
        Constants.AlgaeConstants.kD
    );

    private SparkMaxConfig algaeConfig;


    public AlgaeSubsystem() {
        algaeMotor = new SparkMax(Constants.AlgaeConstants.algaeMotorID, SparkMax.MotorType.kBrushless);
        algaeEncoder = new CANcoder(Constants.AlgaeConstants.algaeEncoderID);

        algaeConfig = new SparkMaxConfig();
        algaeConfig.inverted(false);
        algaeConfig.idleMode(IdleMode.kBrake);

        algaeMotor.configure(algaeConfig, null, null);

        algaeSpeed = 0.0;
        SmartDashboard.putNumber("Algae P", Constants.AlgaeConstants.kP);
        SmartDashboard.putNumber("Algae I", Constants.AlgaeConstants.kI);
        SmartDashboard.putNumber("Algae D", Constants.AlgaeConstants.kD);
    }

    @Override
    public void periodic() {

        algaePID.setPID(
            SmartDashboard.getNumber("Algae P", Constants.AlgaeConstants.kP),
            SmartDashboard.getNumber("Algae I", Constants.AlgaeConstants.kI),
            SmartDashboard.getNumber("Algae D", Constants.AlgaeConstants.kD)
        );
        algaePID.setSetpoint(algaeSetpoint);
        algaeEncoderPosition = algaeEncoder.getPosition().getValueAsDouble();

        if (hold) {
            setCurSetpoint();
            algaeSpeed = algaePID.calculate(algaeEncoderPosition);
            algaeSpeed = MathUtil.clamp(algaeSpeed, -Constants.AlgaeConstants.algaeMaxSpeed, Constants.AlgaeConstants.algaeMaxSpeed);
        }
        
        algaeMotor.set(algaeSpeed);

    }

    public void setIntakeSpeed() {
        hold = false;
        algaeSpeed = Constants.AlgaeConstants.algaeMaxSpeed;
    }

    public void setOuttakeSpeed() {
        hold = false;
        algaeSpeed = -Constants.AlgaeConstants.algaeMaxSpeed;
    }

    public void setCurSetpoint() {
        if (!hasReset) {
            algaeSetpoint = algaeEncoderPosition;
            hasReset = true;
        }
    }

    public void setHold() {
        hold = true;
        hasReset = false;
        algaeSpeed = 0.0;
    }
}
