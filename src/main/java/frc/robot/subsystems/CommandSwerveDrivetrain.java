package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;

import java.util.Optional;
import java.util.function.Supplier;

import com.ctre.phoenix6.Orchestra;
import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.configs.AudioConfigs;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.util.DriveFeedforwards;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.swerve.SwerveModule;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.DoubleArrayPublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;
import frc.robot.generated.TunerConstants;
import frc.robot.generated.TunerConstants.TunerSwerveDrivetrain;

/**
 * Class that extends the Phoenix 6 SwerveDrivetrain class and implements
 * Subsystem so it can easily be used in command-based projects.
 */
public class CommandSwerveDrivetrain extends TunerSwerveDrivetrain implements Subsystem {
    private static final double kSimLoopPeriod = 0.005; // 5 ms
    private Notifier m_simNotifier = null;
    private double m_lastSimTime;

    /* Blue alliance sees forward as 0 degrees (toward red alliance wall) */
    private static final Rotation2d kBlueAlliancePerspectiveRotation = Rotation2d.kZero;
    /* Red alliance sees forward as 180 degrees (toward blue alliance wall) */
    private static final Rotation2d kRedAlliancePerspectiveRotation = Rotation2d.k180deg;
    /* Keep track if we've ever applied the operator perspective before or not */
    private boolean m_hasAppliedOperatorPerspective = false;

    /* Swerve requests to apply during SysId characterization */
    private final SwerveRequest.SysIdSwerveTranslation m_translationCharacterization = new SwerveRequest.SysIdSwerveTranslation();
    private final SwerveRequest.SysIdSwerveSteerGains m_steerCharacterization = new SwerveRequest.SysIdSwerveSteerGains();
    private final SwerveRequest.SysIdSwerveRotation m_rotationCharacterization = new SwerveRequest.SysIdSwerveRotation();

    public SwerveDrivePoseEstimator PoseEstimator;
    public double[] pose;
    public double poseX;
    public double poseY;
    public Rotation2d poseR;
    public double timeStamp;

    private final NetworkTableInstance inst = NetworkTableInstance.getDefault();

    private final NetworkTable table = inst.getTable("Pose");
    private final DoubleArrayPublisher fieldPub = table.getDoubleArrayTopic("robotPose").publish();
    private final StringPublisher fieldTypePub = table.getStringTopic(".type").publish();

    public Pigeon2 gyro;
    public RobotConfig config;

    public SwerveRequest request;
    public boolean isPigeonInitialized = false;

    public StructPublisher<Pose2d> publisher;

    public int lastUpdate = 0;

    private double speed;
    private double angularSpeed;

    public Orchestra orchestra = new Orchestra();

    public SwerveModulePosition[] getModulePositions() {
        return new SwerveModulePosition[] {
            this.getModule(0).getPosition(false),
            this.getModule(1).getPosition(false),
            this.getModule(2).getPosition(false),
            this.getModule(3).getPosition(false)
        };  
    }

    /*
     * SysId routine for characterizing translation. This is used to find PID gains
     * for the drive motors.
     */
    private final SysIdRoutine m_sysIdRoutineTranslation = new SysIdRoutine(
            new SysIdRoutine.Config(
                    null, // Use default ramp rate (1 V/s)
                    Volts.of(4), // Reduce dynamic step voltage to 4 V to prevent brownout
                    null, // Use default timeout (10 s)
                    // Log state with SignalLogger class
                    state -> SignalLogger.writeString("SysIdTranslation_State", state.toString())),
            new SysIdRoutine.Mechanism(
                    output -> setControl(m_translationCharacterization.withVolts(output)),
                    null,
                    this));

    /*
     * SysId routine for characterizing steer. This is used to find PID gains for
     * the steer motors.
     */
    private final SysIdRoutine m_sysIdRoutineSteer = new SysIdRoutine(
            new SysIdRoutine.Config(
                    null, // Use default ramp rate (1 V/s)
                    Volts.of(7), // Use dynamic voltage of 7 V
                    null, // Use default timeout (10 s)
                    // Log state with SignalLogger class
                    state -> SignalLogger.writeString("SysIdSteer_State", state.toString())),
            new SysIdRoutine.Mechanism(
                    volts -> setControl(m_steerCharacterization.withVolts(volts)),
                    null,
                    this));

    /*
     * SysId routine for characterizing rotation.
     * This is used to find PID gains for the FieldCentricFacingAngle
     * HeadingController.
     * See the documentation of SwerveRequest.SysIdSwerveRotation for info on
     * importing the log to SysId.
     */
    private final SysIdRoutine m_sysIdRoutineRotation = new SysIdRoutine(
            new SysIdRoutine.Config(
                    /* This is in radians per second², but SysId only supports "volts per second" */
                    Volts.of(Math.PI / 6).per(Second),
                    /* This is in radians per second, but SysId only supports "volts" */
                    Volts.of(Math.PI),
                    null, // Use default timeout (10 s)
                    // Log state with SignalLogger class
                    state -> SignalLogger.writeString("SysIdRotation_State", state.toString())),
            new SysIdRoutine.Mechanism(
                    output -> {
                        /* output is actually radians per second, but SysId only supports "volts" */
                        setControl(m_rotationCharacterization.withRotationalRate(output.in(Volts)));
                        /* also log the requested output for SysId */
                        SignalLogger.writeDouble("Rotational_Rate", output.in(Volts));
                    },
                    null,
                    this));

    /* The SysId routine to test */
    private SysIdRoutine m_sysIdRoutineToApply = m_sysIdRoutineTranslation;

    /**
     * Constructs a CTRE SwerveDrivetrain using the specified constants.
     * <p>
     * This constructs the underlying hardware devices, so users should not
     * construct
     * the devices themselves. If they need the devices, they can access them
     * through
     * getters in the classes.
     *
     * @param drivetrainConstants Drivetrain-wide constants for the swerve drive
     * @param modules             Constants for each specific module
     */
    public CommandSwerveDrivetrain(
            SwerveDrivetrainConstants drivetrainConstants,
            SwerveModuleConstants<?, ?, ?>... modules) {
        super(drivetrainConstants, modules);
        if (Utils.isSimulation()) {
            startSimThread();
        }
        setPathPlanner();

        

        PoseEstimator = new SwerveDrivePoseEstimator(
                this.getKinematics(),
                this.getPigeon2().getRotation2d(),
                getModulePositions(),
                new Pose2d(new Translation2d(0, 0), new Rotation2d(0)));
        gyro = this.getPigeon2();

        publisher = NetworkTableInstance.getDefault()
        .getStructTopic("AdPose", Pose2d.struct).publish();
        
    }

    /**
     * Constructs a CTRE SwerveDrivetrain using the specified constants.
     * <p>
     * This constructs the underlying hardware devices, so users should not
     * construct
     * the devices themselves. If they need the devices, they can access them
     * through
     * getters in the classes.
     *
     * @param drivetrainConstants     Drivetrain-wide constants for the swerve drive
     * @param odometryUpdateFrequency The frequency to run the odometry loop. If
     *                                unspecified or set to 0 Hz, this is 250 Hz on
     *                                CAN FD, and 100 Hz on CAN 2.0.
     * @param modules                 Constants for each specific module
     */
    public CommandSwerveDrivetrain(
            SwerveDrivetrainConstants drivetrainConstants,
            double odometryUpdateFrequency,
            SwerveModuleConstants<?, ?, ?>... modules) {
        super(drivetrainConstants, odometryUpdateFrequency, modules);
        if (Utils.isSimulation()) {
            startSimThread();
        }

        PoseEstimator = new SwerveDrivePoseEstimator(
                this.getKinematics(),
                this.getPigeon2().getRotation2d(),
                getModulePositions(),
                new Pose2d(new Translation2d(0, 0), new Rotation2d(0)));
        gyro = this.getPigeon2();
        setPathPlanner();
        publisher = NetworkTableInstance.getDefault()
        .getStructTopic("Pose", Pose2d.struct).publish();


    }

    /**
     * Constructs a CTRE SwerveDrivetrain using the specified constants.
     * <p>
     * This constructs the underlying hardware devices, so users should not
     * construct
     * the devices themselves. If they need the devices, they can access them
     * through
     * getters in the classes.
     *
     * @param drivetrainConstants       Drivetrain-wide constants for the swerve
     *                                  drive
     * @param odometryUpdateFrequency   The frequency to run the odometry loop. If
     *                                  unspecified or set to 0 Hz, this is 250 Hz
     *                                  on
     *                                  CAN FD, and 100 Hz on CAN 2.0.
     * @param odometryStandardDeviation The standard deviation for odometry
     *                                  calculation
     *                                  in the form [x, y, theta]ᵀ, with units in
     *                                  meters
     *                                  and radians
     * @param visionStandardDeviation   The standard deviation for vision
     *                                  calculation
     *                                  in the form [x, y, theta]ᵀ, with units in
     *                                  meters
     *                                  and radians
     * @param modules                   Constants for each specific module
     */
    public CommandSwerveDrivetrain(
            SwerveDrivetrainConstants drivetrainConstants,
            double odometryUpdateFrequency,
            Matrix<N3, N1> odometryStandardDeviation,
            Matrix<N3, N1> visionStandardDeviation,
            SwerveModuleConstants<?, ?, ?>... modules) {
        super(drivetrainConstants, odometryUpdateFrequency, odometryStandardDeviation, visionStandardDeviation,
                modules);
        if (Utils.isSimulation()) {
            startSimThread();
            
        }
        setPathPlanner();


        PoseEstimator = new SwerveDrivePoseEstimator(
                this.getKinematics(),
                this.getPigeon2().getRotation2d(),
                getModulePositions(),
                new Pose2d(new Translation2d(0, 0), new Rotation2d(0)));

        gyro = this.getPigeon2();

        publisher = NetworkTableInstance.getDefault()
        .getStructTopic("Pose", Pose2d.struct).publish();

        
    }

    /**
     * Returns a command that applies the specified control request to this swerve
     * drivetrain.
     *
     * @param request Function returning the request to apply
     * @return Command to run
     */
    public Command applyRequest(Supplier<SwerveRequest> requestSupplier) {
        return run(() -> this.setControl(requestSupplier.get()));
    }

    /**
     * Runs the SysId Quasistatic test in the given direction for the routine
     * specified by {@link #m_sysIdRoutineToApply}.
     *
     * @param direction Direction of the SysId Quasistatic test
     * @return Command to run
     */
    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return m_sysIdRoutineToApply.quasistatic(direction);
    }

    /**
     * Runs the SysId Dynamic test in the given direction for the routine
     * specified by {@link #m_sysIdRoutineToApply}.
     *
     * @param direction Direction of the SysId Dynamic test
     * @return Command to run
     */
    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return m_sysIdRoutineToApply.dynamic(direction);
    }

    public Pose2d getPose() {
        return PoseEstimator.getEstimatedPosition();

    }

    public void resetPose(Pose2d pose) {
        PoseEstimator.resetPose(pose);
    }

    public void zeroGyro(){
        pose = NetworkTableInstance.getDefault().getTable("limelight").getEntry("botpose_wpiblue").getDoubleArray(new double[6]);
        Rotation2d poseR = Rotation2d.fromDegrees(pose[5]);
          if (Math.abs(pose[0]) >= 0.1) {
              gyro.setYaw(poseR.getDegrees(),0.0);
              System.out.println("Zeroed GYRO *********** ************ *******");
          }

    }

    public void checkFirstZeroGyro() {
        double speed = Math.sqrt(Math.pow(this.getState().Speeds.vxMetersPerSecond,2)+
                                Math.pow(this.getState().Speeds.vxMetersPerSecond,2));
        double angularSpeed = Math.abs(this.getState().Speeds.omegaRadiansPerSecond);
        double ta = inst.getTable("limelight").getEntry("ta").getDouble(0);
        double tx = Math.abs(inst.getTable("limelight").getEntry("tx").getDouble(0));

        if (!(ta < Constants.SwerveConstants.LimelightConstants.taMin ||
            tx > Constants.SwerveConstants.LimelightConstants.txMin ||
            angularSpeed > Constants.SwerveConstants.LimelightConstants.angularMin ||
            speed > Constants.SwerveConstants.LimelightConstants.speedMin)) {
            zeroGyro();
            System.out.println("MANUAL ZERO GYRO *********** ************ *******");
        }
    }

    public void setSpeeds(ChassisSpeeds speed) {
        request = new SwerveRequest.ApplyChassisSpeeds().withSpeeds(speed);
        this.setControl(request);

    }

    public ChassisSpeeds getChassisSpeeds() {
        return this.getState().Speeds;
    }

    private void setPathPlanner() {
        
        // Load the RobotConfig from the GUI settings. You should probably
        // store this in your Constants file
        try{
        config = RobotConfig.fromGUISettings();
        } catch (Exception e) {
        // Handle exception as needed
        e.printStackTrace();
        }

        // Configure AutoBuilder last
        AutoBuilder.configure(
                this::getPose, // Robot pose supplier
                this::resetPose, // Method to reset odometry (will be called if your auto has a starting pose)
                this::getChassisSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
                this::setSpeeds, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds. Also optionally outputs individual module feedforwards
                new PPHolonomicDriveController( // PPHolonomicController is the built in path following controller for holonomic drive trains
                        new PIDConstants(3.0, 0.0, 0.4), // Translation PID constants
                        new PIDConstants(3.5, 0.0, 0.17) // Rotation PID constants
                ),
                config, // The robot configuration
                () -> {
                // Boolean supplier that controls when the path will be mirrored for the red alliance
                // This will flip the path being followed to the red side of the field.
                // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

                Optional<DriverStation.Alliance> alliance = DriverStation.getAlliance();
                if (alliance.isPresent()) {
                    return alliance.get() == DriverStation.Alliance.Red;
                }
                return false;
                },
                this // Reference to this subsystem to set requirements
        );
    }

    public void updateGyroPeriodically() {

        double speed = Math.sqrt(Math.pow(this.getState().Speeds.vxMetersPerSecond,2)+
                                Math.pow(this.getState().Speeds.vxMetersPerSecond,2));
        double angularSpeed = Math.abs(this.getState().Speeds.omegaRadiansPerSecond);
        double ta = inst.getTable("limelight").getEntry("ta").getDouble(0);
        double tx = Math.abs(inst.getTable("limelight").getEntry("tx").getDouble(0));

        int currentTens = (int) (Timer.getFPGATimestamp() / 15);
        if (currentTens != lastUpdate &&
            !(ta < Constants.SwerveConstants.LimelightConstants.taMin ||
            tx > Constants.SwerveConstants.LimelightConstants.txMin ||
            angularSpeed > Constants.SwerveConstants.LimelightConstants.angularMin ||
            speed > Constants.SwerveConstants.LimelightConstants.speedMin)) {
            zeroGyro();
            lastUpdate = currentTens;
            System.out.println("Updated GYRO PERIODICALLY *********** ************ *******");
        }
    }

    @Override
    public void periodic() {
        // SmartDashboard.putNumberArray("SwerveDrivePositions angles", new double[] {getModulePositions()[0].angle.getRadians(), getModulePositions()[1].angle.getRadians(), getModulePositions()[2].angle.getRadians(), getModulePositions()[3].angle.getRadians()});
        // SmartDashboard.putNumberArray("SwerveDrivePositions DistanceMeteres array", new double[] {getModulePositions()[0].distanceMeters, getModulePositions()[1].distanceMeters, getModulePositions()[2].distanceMeters, getModulePositions()[3].distanceMeters});
        /*
         * Periodically try to apply the operator perspective.
         * If we haven't applied the operator perspective before, then we should apply
         * it regardless of DS state.
         * This allows us to correct the perspective in case the robot code restarts
         * mid-match.
         * Otherwise, only check and apply the operator perspective if the DS is
         * disabled.
         * This ensures driving behavior doesn't change until an explicit disable event
         * occurs during testing.
         */

        try {
            pose = inst.getTable("limelight").getEntry("botpose_orb_wpiblue").getDoubleArray(new double[10]);
            LimelightHelpers.SetRobotOrientation("limelight", gyro.getRotation2d().getDegrees(), 0.0, 0.0, 0.0, 0.0, 0.0);            
            poseX = pose[0];
            poseY = pose[1];
            poseR = gyro.getRotation2d();
            timeStamp = Timer.getFPGATimestamp() - (pose[6] / 1000.0);
            // SmartDashboard.putBoolean("Limelight Status", true);
            Pose2d visionBotPose = new Pose2d(poseX, poseY, poseR);

            // distance from current pose to vision estimated pose
            // double poseDifference = PoseEstimator.getEstimatedPosition().getTranslation()
            //         .getDistance(visionBotPose.getTranslation());

            if (Math.abs(pose[0]) >= 0.1) {
                
                updateGyroPeriodically();
            //     // multiple targets detected
            //     if (pose[7] >= 2) {
            //         if (!DriverStation.isEnabled()) {
            //             gyro.setYaw(poseR.getDegrees());
            //         }
            //         xyStds = 0.5;
            //         degStds = 6;
            //     }
            //     // 1 target with large area and close to estimated pose
            //     else if (pose[9] > 0.8 && poseDifference < 0.5) {
            //         xyStds = 1.0;
            //         degStds = 12;
            //     }
            //     // 1 target farther away and estimated pose is close
            //     else if (pose[9] > 0.1 && poseDifference < 0.3) {
            //         xyStds = 2.0;
            //         degStds = 30;
            //     }
            //     // conditions don't match to add a vision measurement
            //     else {
            //         return;
            //     }
                if (!isPigeonInitialized) {
                    zeroGyro();
                    isPigeonInitialized = true;
                }
                PoseEstimator.addVisionMeasurement(visionBotPose, timeStamp);
            
            }
            PoseEstimator.update(gyro.getRotation2d(), getModulePositions());
        } catch (Exception e) {
            DriverStation.reportError("LIMELIGHT FAIL: RESTART ROBOT CODE", e.getStackTrace());
            // SmartDashboard.putBoolean("Limelight Status", false);
        }

        fieldTypePub.set("Field2d");
        fieldPub.set(new double[] { getPose().getX(), getPose().getY(), getPose().getRotation().getDegrees() });
        publisher.set(PoseEstimator.getEstimatedPosition());


        if (!m_hasAppliedOperatorPerspective || DriverStation.isDisabled()) {
            DriverStation.getAlliance().ifPresent(allianceColor -> {
                setOperatorPerspectiveForward(
                        allianceColor == Alliance.Red
                                ? kRedAlliancePerspectiveRotation
                                : kBlueAlliancePerspectiveRotation);
                m_hasAppliedOperatorPerspective = true;
            });
        }
    }

    private void startSimThread() {
        m_lastSimTime = Utils.getCurrentTimeSeconds();

        /* Run simulation at a faster rate so PID gains behave more reasonably */
        m_simNotifier = new Notifier(() -> {
            final double currentTime = Utils.getCurrentTimeSeconds();
            double deltaTime = currentTime - m_lastSimTime;
            m_lastSimTime = currentTime;

            /* use the measured time delta, get battery voltage from WPILib */
            updateSimState(deltaTime, RobotController.getBatteryVoltage());
        });
        m_simNotifier.startPeriodic(kSimLoopPeriod);
    }

    /**
     * Adds a vision measurement to the Kalman Filter. This will correct the
     * odometry pose estimate
     * while still accounting for measurement noise.
     *
     * @param visionRobotPoseMeters The pose of the robot as measured by the vision
     *                              camera.
     * @param timestampSeconds      The timestamp of the vision measurement in
     *                              seconds.
     */
    @Override
    public void addVisionMeasurement(Pose2d visionRobotPoseMeters, double timestampSeconds) {
        super.addVisionMeasurement(visionRobotPoseMeters, Utils.fpgaToCurrentTime(timestampSeconds));
    }

    /**
     * Adds a vision measurement to the Kalman Filter. This will correct the
     * odometry pose estimate
     * while still accounting for measurement noise.
     * <p>
     * Note that the vision measurement standard deviations passed into this method
     * will continue to apply to future measurements until a subsequent call to
     * {@link #setVisionMeasurementStdDevs(Matrix)} or this method.
     *
     * @param visionRobotPoseMeters    The pose of the robot as measured by the
     *                                 vision camera.
     * @param timestampSeconds         The timestamp of the vision measurement in
     *                                 seconds.
     * @param visionMeasurementStdDevs Standard deviations of the vision pose
     *                                 measurement
     *                                 in the form [x, y, theta]ᵀ, with units in
     *                                 meters and radians.
     */
    @Override
    public void addVisionMeasurement(
            Pose2d visionRobotPoseMeters,
            double timestampSeconds,
            Matrix<N3, N1> visionMeasurementStdDevs) {
        super.addVisionMeasurement(visionRobotPoseMeters, Utils.fpgaToCurrentTime(timestampSeconds),
                visionMeasurementStdDevs);
    }
}
