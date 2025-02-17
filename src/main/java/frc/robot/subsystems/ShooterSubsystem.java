package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import au.grapplerobotics.CanBridge;
import au.grapplerobotics.LaserCan;

import com.revrobotics.spark.SparkMax;
import frc.robot.Constants;

public class ShooterSubsystem extends SubsystemBase { 
    private SparkMax leftShooterMotor;
    private SparkMax rightShooterMotor;

    private SparkMaxConfig leftConfig;
    private SparkMaxConfig rightConfig;

    public double leftShooterSpeed;
    public double rightShooterSpeed;

    private LaserCan laserCan;

    public ShooterSubsystem() {
        leftShooterMotor = new SparkMax(Constants.ShooterConstants.leftShooterMotorID, MotorType.kBrushless);
        rightShooterMotor = new SparkMax(Constants.ShooterConstants.rightShooterMotorID, MotorType.kBrushless);

        laserCan = new LaserCan(33);
        CanBridge.runTCP();

        leftConfig = new SparkMaxConfig();
        rightConfig = new SparkMaxConfig();

        leftConfig.inverted(false);
        rightConfig.inverted(true);

        leftConfig.idleMode(IdleMode.kBrake);
        rightConfig.idleMode(IdleMode.kBrake);

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
