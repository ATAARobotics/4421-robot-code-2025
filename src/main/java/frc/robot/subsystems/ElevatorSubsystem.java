package frc.robot.subsystems;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ElevatorSubsystem extends SubsystemBase {
    private double elevatorSpeed = 0;

    public SparkFlex leftClimbMotor = new SparkFlex(21, MotorType.kBrushless);
    public SparkFlex rightClimbMotor = new SparkFlex(22, MotorType.kBrushless); // Put IDs in Constants.java

    public ElevatorSubsystem() {

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