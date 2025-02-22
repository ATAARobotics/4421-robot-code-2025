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

    /* Controller buttons
     * 
     * operatorJoystick.b: toggle absolute heading mode
     * operatorJoystick.y: toggle slow mode
     * operatorJoystick.povRight: while true set swerve module directions to 0
     * joystick.leftBumper: reset field-centric heading
     * joystick.y: climb up while held
     * joystick.a: climb down while held
     * operatorJoystick.a: zero elevator
     * joystick.x: elevator up with hold PID
     * joystick.b: elevator down with hold PID
     * joystick.povDown: set elevator setpoint to L1
     * joystick.povUp: set elevator setpoint to L2
     * joystick.povLeft: set elevator setpoint to L3
     * joystick.povRight: set elevator setpoint to L4
     * joystick.button(10): set elevator setpoint to intake
     * operatorJoystick.x: set elevator setpoint to rest
     * operatorJoystick.rightBumper: stop elevator
     * operatorJoystick.leftBumper: zero gyro
     * operatorJoystick.povUp and operatorJoystick.a: activate alignment when held
     * joystick.rightBumper: shoot
     * joystick.rightTrigger: intake while trigger held
     * 
     */
  }

  public static class ElevatorConstants {
    public static final int leftClimbMotorID = 21;
    public static final int rightClimbMotorID = 22;
    
    public static final int minLimitTouchLeftPin = 5;
    public static final double maxElevatorSpeed = 0.50;

    public static final double kP = 2.0;
    public static final double kI = 0;
    public static final double kD = 0;

    public static class Encoder {
      public static final int encoderID = 23;

      public static final double offset = 0.042;

      public static final double rest = 0.01;
      public static final double Intake = 0.36 - offset;
      public static final double L1 = 1f - offset;
      public static final double L2 = 1.55 - offset;
      public static final double L3 = 2.4638 - offset;
      public static final double L4 = 4.026 - offset;
      public static final double top = 4.1 - offset;

    }

    public static double threshold = 0.1;
  }

  public static class ClimbConstants {
    public static final int leftClimbMotorID = 41;
    public static final int rightClimbMotorID = 42;

    public static final double maxClimbSpeed = 0.4;

    public static final double kP = 0.1;
    public static final double kI = 0;
    public static final double kD = 0;
  }

  public static class ShooterConstants {
    
    public static final int leftShooterMotorID = 31;
    public static final int rightShooterMotorID = 32;

    public static final double maxShooterSpeed = 0.34;
    public static final double intakeSpeed = 0.2;
    public static final double fractionalRatio = 2; //By how much the speed is divided when shooting L1

    public static final int intakeThreshold = 90;
    public static final int absentThreshold = 30;
  } // 0.2758

  public static class SwerveConstants {
    public static final double maxSpeed = 3.6; // Meters per second, for running auto-alignment
    public static final double maxAngularRate = 3; //99 Rotations per second, for running auto-alignment

    public static final double MaxAngularSpeed = 1.5 * Math.PI;

    public static final double fieldX = 17.548225;
    public static final double fieldY = 8.0518000;
    
  }
   
  
}
