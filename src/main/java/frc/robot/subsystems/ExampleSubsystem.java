// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ExampleSubsystem extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */
  public ExampleSubsystem() {}
  SparkFlex leftClimb = new SparkFlex(41, MotorType.kBrushless);
  SparkFlex rightClimb = new SparkFlex(42, MotorType.kBrushless);
  /**
   * Example command factory method.
   *
   * @return a command
   */
  public void climbUp() {
    leftClimb.set(0.4);
    rightClimb.set(0.4);
    System.out.println(".");
  }
  public void climbDown() {
    leftClimb.set(-0.4);
    rightClimb.set(-0.4);
  }
  public void stop() {
    leftClimb.set(0.0);
    rightClimb.set(0.0);
  }
  public Command exampleMethodCommand() {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(
        () -> {
          /* one-time action goes here */
        });
  }

  /**
   * An example method querying a boolean state of the subsystem (for example, a digital sensor).
   *
   * @return value of some boolean subsystem state, such as a digital sensor.
   */
  public boolean exampleCondition() {
    // Query some boolean state, such as a digital sensor.
    return false;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
