package frc.robot.subsystems;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ElevatorSubsystem extends SubsystemBase {
    private double elevatorSpeed = 0;

    public SparkFlex leftClimbMotor = new SparkFlex(Constants.ElevatorConstants.leftClimbMotorID, MotorType.kBrushless);
    public SparkFlex rightClimbMotor = new SparkFlex(Constants.ElevatorConstants.rightClimbMotorID, MotorType.kBrushless); // Put IDs in Constants.java

    public SparkFlexConfig leftConfig;
    public SparkFlexConfig rightConfig;

    public ElevatorSubsystem() {
        leftConfig = new SparkFlexConfig();
        rightConfig = new SparkFlexConfig();

        leftConfig.idleMode(IdleMode.kBrake);
        rightConfig.idleMode(IdleMode.kBrake);

        leftConfig.inverted(true);
        rightConfig.inverted(false);

        leftClimbMotor.configure(leftConfig, null, null);
        rightClimbMotor.configure(rightConfig, null, null);
    }

    @Override
    public void periodic() {
        leftClimbMotor.set(elevatorSpeed);
        rightClimbMotor.set(elevatorSpeed); // Switch to enum once encoder work
    }

    public void elevatorUp() {
        elevatorSpeed = 0.1;
    }

    public void elevatorDown() {
        elevatorSpeed = -0.1; 
    }

    public void elevatorStop() {
        elevatorSpeed = 0.0;
    }
}