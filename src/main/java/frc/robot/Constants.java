// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;

  }

  public static class ElevatorConstants {
    public static final int leftClimbMotorID = 21;
    public static final int rightClimbMotorID = 22;
    
    public static final int leftClimbMotorEncoderID = 23;
    
    public static final int minLimitTouchLeftPin = 5;
    public static final int minLimitTouchRightPin = 9;

    public static final double maxElevatorSpeed = 0.175;

    public static final double kP = 3.0;
    public static final double kI = 0;
    public static final double kD = 0;

    public static class Encoder {
      public static final int encoderID = 23;

      public static final double rest = 0.02;
      public static final double Intake = 0.5;
      public static final double L1 = 1f;
      public static final double L2 = 1.399169921875;
      public static final double L3 = 2.37;
      public static final double L4 = 3.9;
      public static final double top = 3.9;

    }
  }

  public static class ClimbConstants {
    public static final int leftClimbMotorID = 41;
    public static final int rightClimbMotorID = 42;

    public static final double maxClimbSpeed = 0.4;
  }

  public static class ShooterConstants {
    public static final int leftShooterMotorID = 31;
    public static final int rightShooterMotorID = 32;

    public static final double maxShooterSpeed = 1;
    public static final double fractionalRatio = 2; //By how much the speed is divided when shooting L1
    public static final int leftClimbMotorEncoderID = 23;
  }
}
