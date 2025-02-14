package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkMax;
import frc.robot.Constants;

public class ShooterSubsystem extends SubsystemBase { 
    private SparkMax leftShooterMotor;
    private SparkMax rightShooterMotor;

    private SparkMaxConfig leftConfig;
    private SparkMaxConfig rightConfig;

    public double leftShooterSpeed;
    public double rightShooterSpeed;

    public ShooterSubsystem() {
        leftShooterMotor = new SparkMax(Constants.ShooterConstants.leftShooterMotorID, MotorType.kBrushless);
        rightShooterMotor = new SparkMax(Constants.ShooterConstants.rightShooterMotorID, MotorType.kBrushless);

        leftConfig = new SparkMaxConfig();
        rightConfig = new SparkMaxConfig();

        leftConfig.inverted(true);
        rightConfig.inverted(false);

        leftShooterMotor.configure(leftConfig, null, null);
        rightShooterMotor.configure(rightConfig, null, null);
    }

    @Override
    public void periodic() {
        leftShooterMotor.set(leftShooterSpeed);
        rightShooterMotor.set(rightShooterSpeed);
    }

    public void shoot() {
        leftShooterSpeed = Constants.ShooterConstants.maxShooterSpeed;
        rightShooterSpeed = Constants.ShooterConstants.maxShooterSpeed;
    }

    public void shootL1() {
        leftShooterSpeed = Constants.ShooterConstants.maxShooterSpeed;
        rightShooterSpeed = Constants.ShooterConstants.maxShooterSpeed / Constants.ShooterConstants.fractionalRatio;
    }

    public void stopShooter() {
        leftShooterSpeed = 0.0;
        rightShooterSpeed = 0.0;
    }
}
