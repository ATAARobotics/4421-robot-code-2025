package frc.robot.subsystems;

import java.util.Optional;

import com.ctre.phoenix6.signals.UpdateModeValue;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.PathPoint;
import com.pathplanner.lib.path.Waypoint;
import com.pathplanner.lib.trajectory.PathPlannerTrajectoryState;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.generated.TunerConstants;

public class AlignmentSubsystem extends SubsystemBase {

    public final CommandSwerveDrivetrain m_Swerve;
    // private static Pose2d goalPose = new Pose2d(4.700446000000001,-0.7196820000000002, new Rotation2d(-1.047197551));
    private static Pose2d goalPose;

    private Pose2d currentPose;
    private double curX;
    private double curY;
    private double curR;

    private double goalX;
    private double goalY;
    private double goalR;

    private PIDController controllerX = new PIDController(3, 0.0, 0.4);
    private PIDController controllerY = new PIDController(3, 0.0, 0.4);
    private PIDController controllerR = new PIDController(3.5, 0.0, 0.17);
        
    private double xOutput;
    private double yOutput;
    private double rOutput;

    private double distanceError;
    private double angleError;
    private boolean isAligned;

    private boolean biasedSide = true;
    private int currentIndex = 0;
    private boolean isLeft = true;

    private boolean isAligningToBarge = false;

        
        
    public AlignmentSubsystem(CommandSwerveDrivetrain m_Swerve) {     
        isAligned = false;   

        this.m_Swerve = m_Swerve;
        currentPose = m_Swerve.getPose();

        curX = currentPose.getX();
        curY = currentPose.getY();
        curR = currentPose.getRotation().getRadians();

        goalPose = new Pose2d(curX + 1, curY, new Rotation2d(curR));

        goalX = goalPose.getX();
        goalY = goalPose.getY();
        goalR = goalPose.getRotation().getRadians();

        controllerR.enableContinuousInput(-Math.PI, Math.PI);


        
    }

    @Override
    public void periodic() {
        isAligned = Math.pow((curX - goalX), 2) + Math.pow((curY - goalY), 2) < Constants.SwerveConstants.alignmentthreshold;

        goalX = goalPose.getX();
        goalY = goalPose.getY();
        goalR = goalPose.getRotation().getRadians();
        

        controllerX.setPID(3.0, 0.0, 0.4);
        controllerY.setPID(3.0, 0.0, 0.4);
        controllerR.setPID(3.5, 0.0, 0.17);

        currentPose = m_Swerve.getPose();

        curX = currentPose.getX();
        curY = currentPose.getY();
        curR = currentPose.getRotation().getRadians();

        distanceError = Math.sqrt(Math.pow(curX-goalX, 2) + 
                                    Math.pow(curY-goalY,2));
        angleError = Math.abs(curR - goalR);

        isAligned = distanceError < Constants.SwerveConstants.distanceThreshold;

        if (isAligningToBarge) {
            updateBargeGoal();
            calcBargePID();
        } else {
        
            calcPID();

        // updateSmartDashboard();

            if (!biasedSide) {
                updateGoal();
            } else {
                updateGoalBiased(isLeft);
            }
    
        }

    }

    public void right() {
        isLeft = false;
    }

    public void left() {
        isLeft = true;
    }

    public void alignBarge() {
        isAligningToBarge = true;
    }

    public void regularAlign() {
        isAligningToBarge = false;
    }

    public void setBiasedSideFalse() {
        biasedSide = false;
    }

    public void setBiasedSideTrue() {
        biasedSide = true;
    }

    public void calcPID() {
        xOutput = MathUtil.clamp(controllerX.calculate(curX, goalX), -Constants.SwerveConstants.autoAlignMaxSpeed, Constants.SwerveConstants.autoAlignMaxSpeed);
        yOutput = MathUtil.clamp(controllerY.calculate(curY, goalY), -Constants.SwerveConstants.autoAlignMaxSpeed, Constants.SwerveConstants.autoAlignMaxSpeed);
        rOutput = MathUtil.clamp(controllerR.calculate(curR, goalR), -Constants.SwerveConstants.autoAlignMaxAngularRate, Constants.SwerveConstants.autoAlignMaxAngularRate);
    }

    public void calcBargePID() {
        xOutput = MathUtil.clamp(controllerX.calculate(curX, goalX), -Constants.SwerveConstants.autoAlignMaxSpeed, Constants.SwerveConstants.autoAlignMaxSpeed);
        yOutput = 0.0;
        rOutput = MathUtil.clamp(controllerR.calculate(curR, goalR), -Constants.SwerveConstants.autoAlignMaxAngularRate, Constants.SwerveConstants.autoAlignMaxAngularRate);
    }

    // public void updateSmartDashboard() {
    //     SmartDashboard.putNumber("Waypoint Index", (int) (angle(curY, curX) / 30));

    //     SmartDashboard.putNumber("Distance Error", distanceError);
    //     SmartDashboard.putNumber("Angle Error", angleError);

    //     SmartDashboard.putBoolean("Aligned", isAligned);
    // }

    public void updateGoal() {
        Optional<DriverStation.Alliance> alliance = DriverStation.getAlliance();
        if (alliance.isPresent()) {
            if (alliance.get() == DriverStation.Alliance.Red) {
                // updateRedGoalPose();
                updateFromPathPlannerGoalPoseRed();
            } else{
                // updateGoalPose();
                updateFromPathPlannerGoalPose();
            }
        } else{
            // updateGoalPose();
            updateFromPathPlannerGoalPoseRed();
        }
    }

    public void updateGoalBiased(boolean isLeft) {
        Optional<DriverStation.Alliance> alliance = DriverStation.getAlliance();
        if (alliance.isPresent()) {
            if (alliance.get() == DriverStation.Alliance.Red) {
                // updateRedGoalPose();
                updateFromPathPlannerGoalPoseRedBiased(isLeft);
            } else{
                // updateGoalPose();
                updateFromPathPlannerGoalPoseBiased(isLeft);
            }
        } else{
            // updateGoalPose();
            updateFromPathPlannerGoalPoseRedBiased(isLeft);
        }
    }

    // public void updateGoalPose() {
    //     Pose2d flipCur = currentPose;
    //     // Pose2d flipCur = flipCoords(currentPose);
        
    //     goalPose = Constants.SwerveConstants.Waypoints.Waypoints[(int) (angle(flipCur.getY(), flipCur.getX()) / 30)];
    //     // goalPose = flipCoords(goalPose);
    // }

    // public void updateRedGoalPose() {
    //     // Pose2d flipCur = currentPose;
    //     Pose2d flipCur = flipCoords(currentPose);
        
    //     goalPose = Constants.SwerveConstants.Waypoints.Waypoints[(int) (angle(flipCur.getY(), flipCur.getX()) / 30)];
    //     goalPose = flipCoords(goalPose);
    // }

    public void updateBargeGoal() {
        Optional<DriverStation.Alliance> alliance = DriverStation.getAlliance();
        if (alliance.isPresent()) {
            if (alliance.get() == DriverStation.Alliance.Red) {
                updateBargeGoalPoseRed();
            } else{
                updateBargeGoalPose();
            }
        } else{
            updateBargeGoalPoseRed();
        }
    }

    public void updateBargeGoalPose() {
        try {
            PathPlannerPath goalPath = PathPlannerPath.fromPathFile("Barge");
            PathPoint goalPoint = goalPath.getPoint(goalPath.getAllPathPoints().size() - 1);
            goalPose = new Pose2d(goalPoint.position.getX(), goalPoint.position.getY(), goalPath.getGoalEndState().rotation()); 
            // If angle is not showing the right angle, it is returning heading and not the angle from goal-end-state
        } catch (Exception e) {
            System.out.println("PATH FROM FILE ***************** ################ **************** ERROR: " + e);
        }
    }

    public void updateBargeGoalPoseRed() {
        try {
            PathPlannerPath goalPath = PathPlannerPath.fromPathFile("Barge");
            PathPoint goalPoint = goalPath.getPoint(goalPath.getAllPathPoints().size() - 1);
            goalPose = new Pose2d(goalPoint.position.getX(), goalPoint.position.getY(), goalPath.getGoalEndState().rotation()); 
            // If angle is not showing the right angle, it is returning heading and not the angle from goal-end-state
        } catch (Exception e) {
            System.out.println("PATH FROM FILE ***************** ################ **************** ERROR: " + e);
        }
        goalPose = flipCoords(goalPose);
    }

    public void updateFromPathPlannerGoalPoseBiased(boolean isLeft) {
        Pose2d flipCur = currentPose;
        
        currentIndex = (int) ((((angle(flipCur.getY(), flipCur.getX()) + 30) % 360) / 60));
        String goalPoseStr = (isLeft) ? Constants.SwerveConstants.Waypoints.teleopWaypointsLeft[currentIndex] : Constants.SwerveConstants.Waypoints.teleopWaypointsRight[currentIndex];
        try {
            PathPlannerPath goalPath = PathPlannerPath.fromPathFile(goalPoseStr);
            PathPoint goalPoint = goalPath.getPoint(goalPath.getAllPathPoints().size() - 1);
            goalPose = new Pose2d(goalPoint.position.getX(), goalPoint.position.getY(), goalPath.getGoalEndState().rotation()); 
            // If angle is not showing the right angle, it is returning heading and not the angle from goal-end-state
        } catch (Exception e) {
            System.out.println("PATH FROM FILE ***************** ################ **************** ERROR: " + e);
        }
    }

    // flipping GOALPOSE may not be needed if PATHPLANNER returns flipped coords already
    public void updateFromPathPlannerGoalPoseRedBiased(boolean isLeft) {
        Pose2d flipCur = flipCoords(currentPose);
        currentIndex = (int) ((((angle(flipCur.getY(), flipCur.getX()) + 30) % 360) / 60));
        String goalPoseStr = (isLeft) ? Constants.SwerveConstants.Waypoints.teleopWaypointsLeft[currentIndex] : Constants.SwerveConstants.Waypoints.teleopWaypointsRight[currentIndex];
        try {
            PathPlannerPath goalPath = PathPlannerPath.fromPathFile(goalPoseStr);
            PathPoint goalPoint = goalPath.getPoint(goalPath.getAllPathPoints().size() - 1);
            goalPose = new Pose2d(goalPoint.position.getX(), goalPoint.position.getY(), goalPath.getGoalEndState().rotation()); // If angle is not showing the right angle, it is returning heading and not the angle from goal-end-state
        } catch (Exception e) {
            System.out.println("PATH FROM FILE ***************** ################ **************** ERROR: " + e);
        }

        goalPose = flipCoords(goalPose);
    }

    public void updateFromPathPlannerGoalPose() {
        Pose2d flipCur = currentPose;
        
        currentIndex = (int) ((angle(flipCur.getY(), flipCur.getX()) / 30));
        String goalPoseStr = Constants.SwerveConstants.Waypoints.teleopWaypoints[currentIndex];
        try {
            PathPlannerPath goalPath = PathPlannerPath.fromPathFile(goalPoseStr);
            PathPoint goalPoint = goalPath.getPoint(goalPath.getAllPathPoints().size() - 1);
            goalPose = new Pose2d(goalPoint.position.getX(), goalPoint.position.getY(), goalPath.getGoalEndState().rotation()); 
            // If angle is not showing the right angle, it is returning heading and not the angle from goal-end-state
        } catch (Exception e) {
            System.out.println("PATH FROM FILE ***************** ################ **************** ERROR: " + e);
        }
    }

    // flipping GOALPOSE may not be needed if PATHPLANNER returns flipped coords already
    public void updateFromPathPlannerGoalPoseRed() {
        Pose2d flipCur = flipCoords(currentPose);
        currentIndex = (int) (angle(flipCur.getY(), flipCur.getX()) / 30);
        String goalPoseStr = Constants.SwerveConstants.Waypoints.teleopWaypoints[currentIndex];
        try {
            PathPlannerPath goalPath = PathPlannerPath.fromPathFile(goalPoseStr);
            PathPoint goalPoint = goalPath.getPoint(goalPath.getAllPathPoints().size() - 1);
            goalPose = new Pose2d(goalPoint.position.getX(), goalPoint.position.getY(), goalPath.getGoalEndState().rotation()); // If angle is not showing the right angle, it is returning heading and not the angle from goal-end-state
        } catch (Exception e) {
            System.out.println("PATH FROM FILE ***************** ################ **************** ERROR: " + e);
        }

        goalPose = flipCoords(goalPose);
    }

    public Pose2d flipCoords(Pose2d pose) {
        double x = pose.getX();
        double y = pose.getY();
        double r = pose.getRotation().getDegrees();

        double transformX = -(x - Constants.SwerveConstants.fieldX) + Constants.SwerveConstants.fieldX;
        double transformY = -(y - Constants.SwerveConstants.fieldY) + Constants.SwerveConstants.fieldY;
        double transformR = (r + 180) % 360;

        return new Pose2d(transformX, transformY, Rotation2d.fromDegrees(transformR));
    }

    public double angle(double y, double x) {
        double reefCoordsX = Constants.SwerveConstants.Waypoints.centerOfReef.getX();
        double reefCoordsY = Constants.SwerveConstants.Waypoints.centerOfReef.getY();

    
        return ((Math.atan2(y - reefCoordsY, x - reefCoordsX) * 180 / Math.PI) + 360) % 360;
    }

    // returns speed of swerve modules
    public double[] getOutputs() {
        Optional<DriverStation.Alliance> alliance = DriverStation.getAlliance();
            if (alliance.isPresent()) {
                if (alliance.get() == DriverStation.Alliance.Red) {
                    return new double[]{-xOutput, -yOutput, rOutput};
                } else{
                    return new double[]{xOutput, yOutput, rOutput};
                }

            } else{
                return new double[]{xOutput, yOutput, rOutput};

            }
    }

    public boolean aligned() {
        return isAligned;
    }
}
