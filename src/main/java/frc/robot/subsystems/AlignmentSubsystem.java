package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class AlignmentSubsystem extends SubsystemBase {

    public final CommandSwerveDrivetrain m_Swerve;
    private static Pose2d goalPose = new Pose2d(0f, 0f, new Rotation2d(0.0));

    private Pose2d currentPose;
    private double curX;
    private double curY;
    private double curR;

    private double goalX;
    private double goalY;
    private double goalR;

    private PIDController controllerXY = new PIDController(0.1, 0.0, 0.0);
    private PIDController controllerR = new PIDController(0.1, 0.0, 0.0);
        
    private double xOutput;
    private double yOutput;
    private double rOutput;
        
    private double controllerXY_P;
    private double controllerXY_I;
    private double controllerXY_D;

    private double controllerR_P;
    private double controllerR_I;
    private double controllerR_D;
        
        
    public AlignmentSubsystem(CommandSwerveDrivetrain m_Swerve) {
        SmartDashboard.putNumber("AlignmentXY P", controllerR.getP());
        SmartDashboard.putNumber("AlignmentXY I", controllerR.getI());
        SmartDashboard.putNumber("AlignmentXY D", controllerR.getD());

        SmartDashboard.putNumber("AlignmentR P", controllerR.getP());
        SmartDashboard.putNumber("AlignmentR I", controllerR.getI());
        SmartDashboard.putNumber("AlignmentR D", controllerR.getD());


        this.m_Swerve = m_Swerve;
        currentPose = m_Swerve.getState().Pose;

        curX = currentPose.getX();
        curY = currentPose.getY();
        curR = currentPose.getRotation().getRadians();

        goalX = goalPose.getX();
        goalY = goalPose.getY();
        goalR = goalPose.getRotation().getRadians();

        controllerR.enableContinuousInput(-Math.PI, Math.PI);

    }

    @Override
    public void periodic() {

        controllerXY_P = SmartDashboard.getNumber("AlignmentXY P", 0.0);
        controllerXY_I = SmartDashboard.getNumber("AlignmentXY I", 0.0);
        controllerXY_D = SmartDashboard.getNumber("AlignmentXY D", 0.0);

        controllerR_P = SmartDashboard.getNumber("AlignmentR P", 0.0);
        controllerR_I = SmartDashboard.getNumber("AlignmentR I", 0.0);
        controllerR_D = SmartDashboard.getNumber("AlignmentR D", 0.0);

        controllerXY.setPID(controllerXY_P, controllerXY_I, controllerXY_D);
        controllerR.setPID(controllerR_P, controllerR_I, controllerR_D);

        currentPose = m_Swerve.getState().Pose;

        curX = currentPose.getX();
        curY = currentPose.getY();
        curR = currentPose.getRotation().getRadians();

        xOutput = controllerXY.calculate(curX, goalX);
        yOutput = controllerXY.calculate(curY, goalY);
        rOutput = controllerR.calculate(curR, goalR);

        SmartDashboard.putNumber("xOutput", xOutput);
        SmartDashboard.putNumber("yOutput", yOutput);
        SmartDashboard.putNumber("rOutput", rOutput);
    }

    public double[] getOutputs() {
        return new double[]{xOutput, yOutput, rOutput};
    }
}
