package frc.robot.subsystems.Swerve;

import static edu.wpi.first.units.Units.*;

import java.util.function.Supplier;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.subsystems.Swerve.TunerConstants.TunerSwerveDrivetrain;

/**
 * Class that extends the Phoenix 6 SwerveDrivetrain class and implements
 * Subsystem so it can easily be used in command-based projects.
 */
public class SwerveDriveSubsystem extends TunerSwerveDrivetrain implements Subsystem {
    private static final double kSimLoopPeriod = 0.005; // 5 ms
    private Notifier m_simNotifier = null;
    private double m_lastSimTime;
    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second
    /* Blue alliance sees forward as 0 degrees (toward red alliance wall) */
    private static final Rotation2d kBlueAlliancePerspectiveRotation = Rotation2d.fromDegrees(0);
    /* Red alliance sees forward as 180 degrees (toward blue alliance wall) */
    private static final Rotation2d kRedAlliancePerspectiveRotation = Rotation2d.fromDegrees(180);
    /* Keep track if we've ever applied the operator perspective before or not */
    private boolean m_hasAppliedOperatorPerspective = false;
    private SysID sysID = new SysID();
    public SwerveCommands Commands = new SwerveCommands();
    private SwerveRequest.ApplyRobotSpeeds autoRequest = new SwerveRequest.ApplyRobotSpeeds();
    Alliance CurrentAlliance = Alliance.Blue;

    public SwerveDriveSubsystem getSubsystem() {
        return this;
    }

    public SwerveDriveSubsystem(SwerveDrivetrainConstants drivetrainConstants,
            SwerveModuleConstants<?, ?, ?>... modules) {
        super(drivetrainConstants, modules);
        if (Utils.isSimulation()) {
            startSimThread();
        }
    }

    public SwerveDriveSubsystem(
            SwerveDrivetrainConstants drivetrainConstants,
            double odometryUpdateFrequency,
            SwerveModuleConstants<?, ?, ?>... modules) {
        super(drivetrainConstants, odometryUpdateFrequency, modules);
        if (Utils.isSimulation()) {
            startSimThread();
        }
    }

    public SwerveDriveSubsystem(
            SwerveDrivetrainConstants drivetrainConstants, double odometryUpdateFrequency,
            Matrix<N3, N1> odometryStandardDeviation, Matrix<N3, N1> visionStandardDeviation,
            SwerveModuleConstants<?, ?, ?>... modules) {
        super(drivetrainConstants, odometryUpdateFrequency, odometryStandardDeviation, visionStandardDeviation,
                modules);
        if (Utils.isSimulation()) {
            startSimThread();
        }
    }

    public void configureAuto() {
        RobotConfig config = null;
        try {
            config = RobotConfig.fromGUISettings();
        } catch (Exception e) {
            // Handle exception as needed
            e.printStackTrace();
        }

        // Configure AutoBuilder last
        AutoBuilder.configure(
                () -> getState().Pose, // Robot pose supplier
                this::resetPose, // Method to reset odometry (will be called if your auto has a starting pose)
                () -> getState().Speeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
                (speeds, feedforwards) -> setControl(
                        autoRequest.withSpeeds(speeds)
                                .withWheelForceFeedforwardsX(feedforwards.robotRelativeForcesXNewtons())
                                .withWheelForceFeedforwardsY(feedforwards.robotRelativeForcesYNewtons())), // RELATIVE ChassisSpeeds. Also optionally outputs individual module feedforwards11
                new PPHolonomicDriveController( // PPHolonomicController is the built in path following controller for
                                                // holonomic drive trains
                        new PIDConstants(5.0, 0.0, 0.0), // Translation PID constants
                        new PIDConstants(3.0, 0.0, 0.0) // Rotation PID constants
                ),
                config, // The robot configuration
                () -> {
                    // Boolean supplier that controls when the path will be mirrored for the red
                    // alliance
                    // This will flip the path being followed to the red side of the field.
                    // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

                    var alliance = DriverStation.getAlliance();
                    if (alliance.isPresent()) {
                        return alliance.get() == DriverStation.Alliance.Red;
                    }
                    return false;
                },
                this // Reference to this subsystem to set requirements
        );
    }

    public class SwerveCommands {

        public Command applyRequest(Supplier<SwerveRequest> requestSupplier) {
            return run(() -> setControl(requestSupplier.get()));
        }

        public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
            return sysID.m_sysIdRoutineToApply.quasistatic(direction);
        }

        public Command sysIdDynamic(SysIdRoutine.Direction direction) {
            return sysID.m_sysIdRoutineToApply.dynamic(direction);
        }

        public Command autoAlign(String AB) {
            PathConstraints constraints = new PathConstraints(TunerConstants.kSpeedAt12Volts.in(MetersPerSecond), 10.2,
                    9, 30);
            int Side = 6;
            if (CurrentAlliance == DriverStation.Alliance.Blue) {
                // blue
                if (getState().Pose.getX() > 4.478) {
                    // right side of blue
                    if (getState().Pose.getY() > (0.6 * (getState().Pose.getX() - 4.478)) + 3.987) {
                        // top right of blue
                        Side = 10;
                    } else if (getState().Pose.getY() < (-0.6 * (getState().Pose.getX() - 4.478)) + 3.987) {
                        // bottom Right of blue
                        Side = 2;
                    } else {
                        Side = 12;
                    }
                } else {
                    // left side of blue
                    if (getState().Pose.getY() > (-0.6 * (getState().Pose.getX() - 4.478)) + 3.987) {
                        // top left of blue
                        Side = 8;
                    } else if (getState().Pose.getY() < (0.6 * (getState().Pose.getX() - 4.478)) + 3.987) {
                        // bottom left of blue
                        Side = 4;
                    } else {
                        Side = 6;
                    }
                }
            } else {
                // red
                if (getState().Pose.getX() < 13.102) {
                    // left side of red
                    if (getState().Pose.getY() > (-0.6 * (getState().Pose.getX() - 13.102)) + 3.987) {
                        // top right of blue
                        Side = 10;
                    } else if (getState().Pose.getY() < (0.6 * (getState().Pose.getX() - 13.102)) + 3.987) {
                        // bottom Right of blue
                        Side = 2;
                    } else {
                        Side = 12;
                    }
                } else {
                    // left side of blue
                    if (getState().Pose.getY() > (0.6 * (getState().Pose.getX() - 13.102)) + 3.987) {
                        // top left of blue
                        Side = 8;
                    } else if (getState().Pose.getY() < (-0.6 * (getState().Pose.getX() - 13.102)) + 3.987) {
                        // bottom left of blue
                        Side = 4;
                    } else {
                        Side = 6;
                    }
                }

            }

            SmartDashboard.putNumber("side", Side);
            try {
                return AutoBuilder.pathfindThenFollowPath(PathPlannerPath.fromPathFile("R" + Side + AB), constraints);
            } catch (Exception e) {
                return new PrintCommand("Path planner path does not exist");
            }
        }
    }

    @Override
    public void periodic() {
        if (!m_hasAppliedOperatorPerspective || DriverStation.isDisabled()) {
            if (Utils.isSimulation()) {
                setOperatorPerspectiveForward(Rotation2d.fromDegrees(90));
            } else {
                DriverStation.getAlliance().ifPresent(allianceColor -> {
                    CurrentAlliance = allianceColor;
                    setOperatorPerspectiveForward(
                            allianceColor == Alliance.Red
                                    ? kRedAlliancePerspectiveRotation
                                    : kBlueAlliancePerspectiveRotation);
                    m_hasAppliedOperatorPerspective = true;
                });
            }
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

    public class SysID {

        /* Swerve requests to apply during SysId characterization */
        private final SwerveRequest.SysIdSwerveTranslation m_translationCharacterization = new SwerveRequest.SysIdSwerveTranslation();
        private final SwerveRequest.SysIdSwerveSteerGains m_steerCharacterization = new SwerveRequest.SysIdSwerveSteerGains();
        private final SwerveRequest.SysIdSwerveRotation m_rotationCharacterization = new SwerveRequest.SysIdSwerveRotation();

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
                        getSubsystem()));

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
                        getSubsystem()));

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
                        getSubsystem()));

        /* The SysId routine to test */
        private SysIdRoutine m_sysIdRoutineToApply = m_sysIdRoutineTranslation;

    }

}
