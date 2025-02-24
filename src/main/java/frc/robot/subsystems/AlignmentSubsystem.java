package frc.robot.subsystems;

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

    private PIDController controllerXY = new PIDController(2, 0.0, 0.0);
    private PIDController controllerR = new PIDController(3, 0.0, 0.0);

    private double xOutput;
    private double yOutput;
    private double rOutput;
        
    private double controllerXY_P;
    private double controllerXY_I;
    private double controllerXY_D;

    private double controllerR_P;
    private double controllerR_I;
    private double controllerR_D;

    public Field2d goalPoseField = new Field2d();
        
        
    public AlignmentSubsystem(CommandSwerveDrivetrain m_Swerve) {        
        SmartDashboard.putNumber("AlignmentXY P", controllerR.getP());
        SmartDashboard.putNumber("AlignmentXY I", controllerR.getI());
        SmartDashboard.putNumber("AlignmentXY D", controllerR.getD());

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
        
        controllerXY_P = SmartDashboard.getNumber("AlignmentXY P", 0.0);
        controllerXY_I = SmartDashboard.getNumber("AlignmentXY I", 0.0);
        controllerXY_D = SmartDashboard.getNumber("AlignmentXY D", 0.0);

        controllerR_P = SmartDashboard.getNumber("AlignmentR P", 0.0);
        controllerR_I = SmartDashboard.getNumber("AlignmentR I", 0.0);
        controllerR_D = SmartDashboard.getNumber("AlignmentR D", 0.0);

        controllerXY.setPID(controllerXY_P, controllerXY_I, controllerXY_D);
        controllerR.setPID(controllerR_P, controllerR_I, controllerR_D);

        currentPose = m_Swerve.getPose();

        curX = currentPose.getX();
        curY = currentPose.getY();
        curR = currentPose.getRotation().getRadians();

        xOutput = MathUtil.clamp(controllerXY.calculate(curX, goalX), -Constants.SwerveConstants.autoAlignMaxSpeed, Constants.SwerveConstants.autoAlignMaxSpeed);
        yOutput = MathUtil.clamp(controllerXY.calculate(curY, goalY), -Constants.SwerveConstants.autoAlignMaxSpeed, Constants.SwerveConstants.autoAlignMaxSpeed);
        rOutput = MathUtil.clamp(controllerR.calculate(curR, goalR), -Constants.SwerveConstants.autoAlignMaxAngularRate, Constants.SwerveConstants.autoAlignMaxAngularRate);

        SmartDashboard.putNumber("xOutput", xOutput);
        SmartDashboard.putNumber("yOutput", yOutput);
        SmartDashboard.putNumber("rOutput", rOutput);

        SmartDashboard.putNumber("Current Robot X", currentPose.getX());
        SmartDashboard.putNumber("Current Robot Y", currentPose.getY());
        SmartDashboard.putNumber("Current Robot R", currentPose.getRotation().getDegrees());
        
        SmartDashboard.putNumber("Waypoint Index", (int) (angle(curY, curX) / 30));

        updateGoalPose();
        goalPoseField.setRobotPose(goalPose);
    }

    public void updateGoalPose() {
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
        return new double[]{xOutput, yOutput, rOutput};
    }
}
