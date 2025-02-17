package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
    private LaserCan shooterLaserCan;

    private int laserCan_distance;
    private int shooterLaserCan_distance;

    private enum ShooterState {
        IDLE, INTAKE, SHOOT, SHOOTL1
    }

    private ShooterState shooterState;
    private double leftSpeed;
    private double rightSpeed;

    public ShooterSubsystem() {
        leftSpeed = 0.0;
        rightSpeed = 0.0;

        shooterState = ShooterState.IDLE;

        laserCan = new LaserCan(33);
        shooterLaserCan = new LaserCan(34);
        leftShooterMotor = new SparkMax(Constants.ShooterConstants.leftShooterMotorID, MotorType.kBrushless);
        rightShooterMotor = new SparkMax(Constants.ShooterConstants.rightShooterMotorID, MotorType.kBrushless);

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
        laserCan_distance = laserCan.getMeasurement().distance_mm;
        shooterLaserCan_distance = shooterLaserCan.getMeasurement().distance_mm;

        SmartDashboard.putNumber("LaserCan Distance", laserCan_distance);
        SmartDashboard.putNumber("ShooterLaserCan Distance", shooterLaserCan_distance);

        switch(shooterState) {
            case IDLE:
                leftSpeed = 0.0;
                rightSpeed = 0.0;

                if (checkLaserCan()) {
                    shooterState = ShooterState.INTAKE;
                }

                break;

            case INTAKE:
                leftSpeed = Constants.ShooterConstants.intakeSpeed;
                rightSpeed = Constants.ShooterConstants.intakeSpeed;

                if (!checkLaserCan()) {
                    shooterState = ShooterState.IDLE;
                }

                break;
            case SHOOT:
                leftSpeed = Constants.ShooterConstants.maxShooterSpeed;
                rightSpeed = Constants.ShooterConstants.maxShooterSpeed;

                if (!checkShooterLaserCan()) {
                    shooterState = ShooterState.IDLE;
                }

                break;
            
            case SHOOTL1:
                leftSpeed = Constants.ShooterConstants.maxShooterSpeed;
                rightSpeed = Constants.ShooterConstants.maxShooterSpeed / Constants.ShooterConstants.fractionalRatio;

                if (!checkShooterLaserCan()) {
                    shooterState = ShooterState.IDLE;
                }

                break;
        }

        runShooter(leftSpeed, rightSpeed);
        
    }

    public void shoot() {
        shooterState = ShooterState.SHOOT;
    }

    public void shootL1() {
        shooterState = ShooterState.SHOOTL1;
    }

    public void stop() {
        shooterState = ShooterState.IDLE;
    }

    public void runShooter(double leftSpeed, double rightSpeed) {
        leftShooterMotor.set(leftSpeed);
        rightShooterMotor.set(rightSpeed);
    }

    public boolean checkLaserCan() {
        return laserCan_distance < Constants.ShooterConstants.intakeThreshold;
    }

    public boolean checkShooterLaserCan() {
        return shooterLaserCan_distance < Constants.ShooterConstants.absentThreshold;
    }
}
