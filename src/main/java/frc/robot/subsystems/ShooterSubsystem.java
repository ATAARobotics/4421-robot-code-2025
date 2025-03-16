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

    public enum ShooterState {
        IDLE, ALGAE_IN, ALGAE_OUT, INTAKE, REVERSE, SLOW, SHOOT, SHOOTL1
    }

    private ShooterState shooterState;
    private double leftSpeed;
    private double rightSpeed;

    private boolean prevPressed;

    private boolean overrideIntake;

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

        leftConfig.inverted(true);
        rightConfig.inverted(false);

        leftConfig.smartCurrentLimit(20);
        rightConfig.smartCurrentLimit(20);

        leftConfig.idleMode(IdleMode.kBrake);
        rightConfig.idleMode(IdleMode.kBrake);

        leftShooterMotor.configure(leftConfig, null, null);
        rightShooterMotor.configure(rightConfig, null, null);

        overrideIntake = false;
        prevPressed = false;
    }

    @Override
    public void periodic() {
        laserCan_distance = laserCan.getMeasurement().distance_mm;
        shooterLaserCan_distance = shooterLaserCan.getMeasurement().distance_mm;

        SmartDashboard.putNumber("LaserCan Distance", laserCan_distance);
        SmartDashboard.putNumber("ShooterLaserCan Distance", shooterLaserCan_distance);
        SmartDashboard.putString("Shooter State", shooterState.toString());

        SmartDashboard.putBoolean("Check LaserCAN", checkLaserCan());
        SmartDashboard.putBoolean("Check Shooter LaserCAN", checkShooterLaserCan());

        SmartDashboard.putString("Shooter State", shooterState.toString());


        switch(shooterState) {
            case IDLE:
                leftSpeed = 0.0;
                rightSpeed = 0.0;

                if (checkLaserCan() && !checkShooterLaserCan()) {
                    shooterState = ShooterState.INTAKE;
                }
                else if(overrideIntake && !checkShooterLaserCan()) {
                    leftSpeed = Constants.ShooterConstants.maxShooterSpeed;
                    rightSpeed = Constants.ShooterConstants.maxShooterSpeed;
                }

                break;

            case ALGAE_IN:
                leftSpeed = -Constants.ShooterConstants.maxShooterSpeed;
                rightSpeed = -Constants.ShooterConstants.maxShooterSpeed;

                break;

            case ALGAE_OUT:
                leftSpeed = Constants.ShooterConstants.maxShooterSpeed;
                rightSpeed = Constants.ShooterConstants.maxShooterSpeed;

                break;

            case INTAKE:
                leftSpeed = Constants.ShooterConstants.maxShooterSpeed / 1.75;
                rightSpeed = Constants.ShooterConstants.maxShooterSpeed / 1.75;

                if (!checkLaserCan() && checkShooterLaserCan()) {
                    shooterState = ShooterState.REVERSE;
                }

                break;
            case REVERSE:
                leftSpeed = -Constants.ShooterConstants.intakeSpeed;
                rightSpeed = -Constants.ShooterConstants.intakeSpeed;

                if (checkLaserCan() && checkShooterLaserCan()) {
                    shooterState = ShooterState.SLOW;
                }

                break;
            case SLOW:
                leftSpeed = Constants.ShooterConstants.intakeSpeed;
                rightSpeed = Constants.ShooterConstants.intakeSpeed;

                if (!checkLaserCan() && checkShooterLaserCan()) {
                    shooterState = ShooterState.IDLE;
                }

                break;
            case SHOOT:
                leftSpeed = Constants.ShooterConstants.maxShooterSpeed;
                rightSpeed = Constants.ShooterConstants.maxShooterSpeed;

                if (!checkShooterLaserCan()) {
                    shooterState = ShooterState.IDLE;
                    overrideIntake = false;
                }
     
                break;
            
            case SHOOTL1:
                leftSpeed = 0.4;
                rightSpeed = Constants.ShooterConstants.maxShooterSpeed / Constants.ShooterConstants.fractionalRatio;

                if (!checkShooterLaserCan()) {
                    shooterState = ShooterState.IDLE;
                    overrideIntake = false;
                }

                break;
        }

        runShooter(leftSpeed, rightSpeed);
        
    }

    public void intake() {
        shooterState = ShooterState.INTAKE;
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

    public void algaeIn() {
        shooterState = ShooterState.ALGAE_IN;
    }

    public void algaeOut() {
        shooterState = ShooterState.ALGAE_OUT;
    }

    public ShooterState getState() {
        return shooterState;
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

    public void toggleOverride() {
        overrideIntake = !overrideIntake;
    }
}
