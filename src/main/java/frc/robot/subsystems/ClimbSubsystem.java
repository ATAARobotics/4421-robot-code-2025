// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ClimbSubsystem extends SubsystemBase {
    private double climbSpeed = 0;

    SparkFlex leftClimb = new SparkFlex(Constants.ClimbConstants.leftClimbMotorID, MotorType.kBrushless);
    SparkFlex rightClimb = new SparkFlex(Constants.ClimbConstants.rightClimbMotorID, MotorType.kBrushless);

    SparkFlexConfig leftConfig;
    SparkFlexConfig rightConfig;

    public ClimbSubsystem() {
        leftConfig = new SparkFlexConfig();
        rightConfig = new SparkFlexConfig();

        leftConfig.inverted(true);
        rightConfig.inverted(false);

        leftClimb.configure(leftConfig, null, null);
        rightClimb.configure(rightConfig, null, null);
    }

    @Override
    public void periodic(){
        leftClimb.set(climbSpeed);
        rightClimb.set(climbSpeed);
    }

    public void climbUp() {
        climbSpeed = Constants.ClimbConstants.maxClimbSpeed;
    }

    public void climbDown() {
        climbSpeed = -Constants.ClimbConstants.maxClimbSpeed;
    }

    public void stop() {
        climbSpeed = 0.0;
    }
}
