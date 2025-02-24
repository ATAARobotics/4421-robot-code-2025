package frc.robot.subsystems;

import java.util.Optional;

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
        
    private double controllerX_P;
    private double controllerX_I;
    private double controllerX_D;

    private double controllerY_P;
    private double controllerY_I;
    private double controllerY_D;

    private double controllerR_P;
    private double controllerR_I;
    private double controllerR_D;

    public Field2d goalPoseField = new Field2d();
        
        
    public AlignmentSubsystem(CommandSwerveDrivetrain m_Swerve) {        
        SmartDashboard.putNumber("AlignmentX P", controllerX.getP());
        SmartDashboard.putNumber("AlignmentX I", controllerX.getI());
        SmartDashboard.putNumber("AlignmentX D", controllerX.getD());

        SmartDashboard.putNumber("AlignmentY P", controllerY.getP());
        SmartDashboard.putNumber("AlignmentY I", controllerY.getI());
        SmartDashboard.putNumber("AlignmentY D", controllerY.getD());

        SmartDashboard.putNumber("AlignmentR P", controllerR.getP());
        SmartDashboard.putNumber("AlignmentR I", controllerR.getI());
        SmartDashboard.putNumber("AlignmentR D", controllerR.getD());

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

        SmartDashboard.putData("Goal Pose", goalPoseField);

    }

    @Override
    public void periodic() {
        goalX = goalPose.getX();
        goalY = goalPose.getY();
        goalR = goalPose.getRotation().getRadians();
        
        controllerX_P = SmartDashboard.getNumber("AlignmentX P", 0.0);
        controllerX_I = SmartDashboard.getNumber("AlignmentX I", 0.0);
        controllerX_D = SmartDashboard.getNumber("AlignmentX D", 0.0);

        controllerY_P = SmartDashboard.getNumber("AlignmentY P", 0.0);
        controllerY_I = SmartDashboard.getNumber("AlignmentY I", 0.0);
        controllerY_D = SmartDashboard.getNumber("AlignmentY D", 0.0);

        controllerR_P = SmartDashboard.getNumber("AlignmentR P", 0.0);
        controllerR_I = SmartDashboard.getNumber("AlignmentR I", 0.0);
        controllerR_D = SmartDashboard.getNumber("AlignmentR D", 0.0);

        controllerX.setPID(controllerX_P, controllerX_I, controllerX_D);
        controllerY.setPID(controllerY_P, controllerY_I, controllerY_D);
        controllerR.setPID(controllerR_P, controllerR_I, controllerR_D);

        currentPose = m_Swerve.getPose();

        curX = currentPose.getX();
        curY = currentPose.getY();
        curR = currentPose.getRotation().getRadians();

        xOutput = MathUtil.clamp(controllerX.calculate(curX, goalX), -Constants.SwerveConstants.autoAlignMaxSpeed, Constants.SwerveConstants.autoAlignMaxSpeed);
        yOutput = MathUtil.clamp(controllerY.calculate(curY, goalY), -Constants.SwerveConstants.autoAlignMaxSpeed, Constants.SwerveConstants.autoAlignMaxSpeed);
        rOutput = MathUtil.clamp(controllerR.calculate(curR, goalR), -Constants.SwerveConstants.autoAlignMaxAngularRate, Constants.SwerveConstants.autoAlignMaxAngularRate);

        SmartDashboard.putNumber("xOutput", xOutput);
        SmartDashboard.putNumber("yOutput", yOutput);
        SmartDashboard.putNumber("rOutput", rOutput);

        SmartDashboard.putNumber("Current Robot X", currentPose.getX());
        SmartDashboard.putNumber("Current Robot Y", currentPose.getY());
        SmartDashboard.putNumber("Current Robot R", currentPose.getRotation().getDegrees());
        
        SmartDashboard.putNumber("Waypoint Index", (int) (angle(curY, curX) / 30));
        Optional<DriverStation.Alliance> alliance = DriverStation.getAlliance();
        if (alliance.isPresent()) {
            updateRedGoalPose();
        } else{
            updateGoalPose();
        }
        goalPoseField.setRobotPose(goalPose);
    }

    public void updateGoalPose() {
        Pose2d flipCur = currentPose;
        // Pose2d flipCur = flipCoords(currentPose);
        
        goalPose = Constants.SwerveConstants.Waypoints.Waypoints[(int) (angle(flipCur.getY(), flipCur.getX()) / 30)];
        // goalPose = flipCoords(goalPose);
    }

    public void updateRedGoalPose() {
        // Pose2d flipCur = currentPose;
        Pose2d flipCur = flipCoords(currentPose);
        
        goalPose = Constants.SwerveConstants.Waypoints.Waypoints[(int) (angle(flipCur.getY(), flipCur.getX()) / 30)];
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
                return new double[]{-xOutput, -yOutput, rOutput};

            } else{
                return new double[]{xOutput, yOutput, rOutput};

            }
    }
}
