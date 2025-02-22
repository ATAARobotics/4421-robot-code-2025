// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.sim.SparkRelativeEncoderSim;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.generated.TunerConstants;

public class ClimbSubsystem extends SubsystemBase {
    private double climbSpeed = 0;

    SparkFlex leftClimb = new SparkFlex(Constants.ClimbConstants.leftClimbMotorID, MotorType.kBrushless);

    SparkFlexConfig leftConfig;
    SparkFlexConfig rightConfig;

    private boolean hold;

    private RelativeEncoder encoder = leftClimb.getEncoder();

    private double curEncoderPosition;
    private double curSetPosition;

    private PIDController climbPID = new PIDController(
        Constants.ClimbConstants.kP,
        Constants.ClimbConstants.kI,
        Constants.ClimbConstants.kD
    );

    public ClimbSubsystem() {
        leftConfig = new SparkFlexConfig();

        leftConfig.inverted(true);

        leftClimb.configure(leftConfig, null, null);

        hold = false;
        curEncoderPosition = encoder.getPosition();
        curSetPosition = curEncoderPosition;
    }

    @Override
    public void periodic(){

        curEncoderPosition = encoder.getPosition();
        if(hold) {
            climbPID.setSetpoint(curSetPosition);
            climbSpeed = MathUtil.clamp(climbPID.calculate(curEncoderPosition), 
                                            -Constants.ClimbConstants.maxClimbSpeed, 
                                            Constants.ClimbConstants.maxClimbSpeed);
        }
        leftClimb.set(climbSpeed);
    }

    public void climbUp() {
        climbSpeed = Constants.ClimbConstants.maxClimbSpeed;
        hold = false;
    }

    public void climbDown() {
        climbSpeed = -Constants.ClimbConstants.maxClimbSpeed;
        hold = false;
    }

    public void stop() {
        climbSpeed = 0.0;
        hold = true;
        curSetPosition = curEncoderPosition;
    }

    public void initHoldMode() {
        hold = false;
    }
}
