// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

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

    // Button mappings
    // DRIVER JOYSTICK
    // y
    // a
    // x
    // b
    // RB
    // LB
    // RT
    // LT
    // 
    // 
    // 
    // 
    // 
    // 
    // 
    // 
    // 
    // 
    // 
    // 
    // 
    //  public static final int kDriverControllerPort = 0;
    //   
  }

  public static class ElevatorConstants {
    public static final int leftClimbMotorID = 21;
    public static final int rightClimbMotorID = 22;
    
    public static final int minLimitTouchLeftPin = 1;
    public static final double maxElevatorSpeed = 0.8;

    public static final double kP = 0.7;
    public static final double kI = 0;
    public static final double kD = 0.06;

    public static final double threshold = 0.1;
    public static final double maxElevatorAcceleration = 1.0;

    public static final double feedForward = 0.0;


    public static class Encoder {
      public static final int encoderID = 23;

      public static final double rest = 0.01;
      public static final double Intake = 0.043;
      public static final double Processor = 1.0;
      public static final double L1 = 0.69;
      public static final double L2 = 1.492;
      public static final double L3 = 2.9;
      public static final double L4 = 5.26;
      public static final double top = 5.27;
      public static final double algaeHigh = 3.18;
      public static final double algaeLow = 1.772;

      public static final double pivotInBetweenPoint = 1.2;
      public static final double pivotL4Point = 4.8;
    }

    public static class Pivot {
      public static final double pivotkP = 5;
      public static final double pivotkI = 0.01;
      public static final double pivotkD = 0.01;

      
      public static final int pivotSparkID = 61;
      public static final int pivotEncoderID = 62;

      public static final double pivotIntake = -0.001;
      public static final double pivotInBetween = 0.015;
      // public static final double pivotL4 = 0.065;
      public static final double pivotClimb = 0.035;
      public static final double pivotL4 = 0.065;
      public static final double pivotShoot = 0.18;
      public static final double pivotAlgae = 0.23;
      public static final double pivotProcessor = 0.3;

      public static final double pivotMaxSpeed = 0.7;

      public static final double pivotThreshold = 0.05;

    }
  }

  public static class ClimbConstants {
    public static final int leftClimbMotorID = 41;
    public static final int rightClimbMotorID = 42;

    public static final double maxClimbSpeed = 0.75;

    public static final double kP = 0.1;
    public static final double kI = 0;
    public static final double kD = 0;
  }

  public static class ShooterConstants {
    
    public static final int leftShooterMotorID = 31;
    public static final int rightShooterMotorID = 32;

    public static final double maxShooterSpeed = 0.32;
    public static final double intakeSpeed = 0.09;
    public static final double fractionalRatio = 3; //By how much the speed is divided when shooting L1

    public static final int intakeThreshold = 80;
    public static final int absentThreshold = 50;
  } // 0.2758


  public static class AlgaeConstants {
    public static final int algaeMotorID = 59;
    public static final int algaeEncoderID = 58;

    public static final double algaeMaxSpeed = 0.75;
    public static final double kP = 1.0;
    public static final double kI = 0;
    public static final double kD = 0;

    public static final double shootStart = 2.0;
  }

  public static class SwerveConstants {
    public static final double alignmentthreshold = 0.05;
    public static final double autoAlignMaxSpeed = 2.0; // Meters per second, for running auto-alignment
    public static final double autoAlignMaxAngularRate = 2.7; //99 Rotations per second, for running auto-alignment

    public static final double autoAlignTimeConstraint = 2.50;
    public static final double shootingCommandWaitCommand = 0.3;

    public static final double MaxAngularSpeed = 1.5 * Math.PI;

    public static final double fieldX = 17.548225 / 2;
    public static final double fieldY = 8.0518000 / 2;

    public static final int gyroCounter = 10;

    public static class LimelightConstants {

      public static final double taMin = 0.001;
      public static final double txMin = 31;
      public static final double angularMin = 0.1 * Math.PI;
      public static final double speedMin = 1;
    }

    public static class Waypoints {
      public static final Pose2d NNW = new Pose2d(5.870, 4.191, Rotation2d.fromDegrees(180.0));
      public static final Pose2d NW = new Pose2d(5.333, 5.156, Rotation2d.  fromDegrees(-120.0));
      public static final Pose2d NWW = new Pose2d(5.119, 5.119, Rotation2d.fromDegrees(-120));

      public static final Pose2d SWW = new Pose2d(3.910, 5.332, Rotation2d.fromDegrees(-60.0));
      public static final Pose2d SW = new Pose2d(3.637, 5.156, Rotation2d.fromDegrees(-60.0));
      public static final Pose2d SSW = new Pose2d(3.101, 4.191, Rotation2d.fromDegrees(0.0));
      
      public static final Pose2d SSE = new Pose2d(3.150, 3.859, Rotation2d.fromDegrees(0.0));
      public static final Pose2d SE = new Pose2d(3.656, 2.904, Rotation2d.fromDegrees(60.0));
      public static final Pose2d SEE = new Pose2d(3.929, 2.757, Rotation2d.fromDegrees(60));

      public static final Pose2d NEE = new Pose2d(5.031, 2.748, Rotation2d.fromDegrees(120));
      public static final Pose2d NE = new Pose2d(5.314, 2.923, Rotation2d.fromDegrees(120.0));
      public static final Pose2d NNE = new Pose2d(5.889, 3.859, Rotation2d.fromDegrees(180.0));

      public static final Pose2d[] Waypoints = {NNW, NW, NWW, SWW, SW, SSW, SSE, SE, SEE, NEE, NE, NNE};
      public static final String[] teleopWaypoints = {"Teleop NNW", "Teleop NW", "Teleop NWW", "Teleop SWW", "Teleop SW", "Teleop SSW", "Teleop SSE", "Teleop SE", "Teleop SEE", "Teleop NEE", "Teleop NE", "Teleop NNE"};
      public static final String[] teleopWaypointsLeft = {"Teleop NNE", "Teleop NW", "Teleop SWW", "Teleop SSW", "Teleop SE", "Teleop NEE"};
      public static final String[] teleopWaypointsRight = {"Teleop NNW", "Teleop NWW", "Teleop SW", "Teleop SSE", "Teleop SEE", "Teleop NE"};

    
      public static final String[] teleopAlignWaypoints = {"Align NNW", "Align NW", "Align NWW", "Align SWW", "Align SW", "Align SSW", "Align SSE", "Align SE", "Align SEE", "Align NEE", "Align NE", "Align NNE"};


      public static final Pose2d centerOfReef = new Pose2d(4.495, 4.015, Rotation2d.fromDegrees(0.0));
    }

    
    
    public static final double linearDeadBand = 0.20;
    public static final double angularDeadBand = 0.15;

    public static final double distanceThreshold = 0.14;
    public static final double angleThreshold = 15 * Math.PI / 180;
  }
  
   
  
}
