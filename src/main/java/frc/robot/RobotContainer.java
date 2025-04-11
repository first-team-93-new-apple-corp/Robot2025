// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import java.util.Set;

import java.util.function.Supplier;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.pathplanner.lib.auto.AutoBuilder;
import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.DeferredCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.subsystems.Controls.ThrottleableDrive;
import frc.robot.subsystems.Controls.TwoStickDriveXboxOp;
import frc.robot.subsystems.Grabber.GrabberSubsystem;
import frc.robot.Constants.Inputs.CameraPipeline;
import frc.robot.Constants.Inputs.Cameras.Camera;
import frc.robot.commands.intake;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.Arm.ArmSubsystem;
import frc.robot.subsystems.Auton.AutoDirector;
import frc.robot.subsystems.Auton.AutoSubsystems;
import frc.robot.subsystems.Controls.ControllerSchemeIO;
// import frc.robot.subsystems.Controls.XboxDrive;
import frc.robot.subsystems.Swerve.SwerveDriveSubsystem;
import frc.robot.subsystems.Swerve.Telemetry;
import frc.robot.subsystems.Swerve.TunerConstants;
import frc.robot.subsystems.VisionIO.CameraFactory;
import frc.robot.subsystems.VisionIO.Vision;

public class RobotContainer {
    // Drivetrain
    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top
                                                                                  // speed
                                                                                  // speed
    public double MaxAngularRate = RadiansPerSecond.of(11.887).in(RadiansPerSecond); // 3/4 of a rotation per second
                                                                                     // max angular velocity
    public final SwerveDriveSubsystem m_DriveSubsystem = TunerConstants.createDrivetrain();
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.02).withRotationalDeadband(MaxAngularRate * 0.02) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.RobotCentric driveRobot = new SwerveRequest.RobotCentric()
            .withDeadband(MaxSpeed * 0.02).withRotationalDeadband(MaxAngularRate * 0.02) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();

    private final ControllerSchemeIO Driver = new TwoStickDriveXboxOp(0, 1, 2);

    // Simulating Elevator
    public ElevatorSubsystem m_ElevatorSubsystem = new ElevatorSubsystem();
    public ArmSubsystem m_ArmSubsystem = new ArmSubsystem();
    // Auton
    AutoDirector autoDirector;
    // Logging
    private final Telemetry logger = new Telemetry(MaxSpeed);
    // Arm subsystem
    // private final ArmSubsystem m_Armsubsystem = new ArmSubsystem();

    // private final Vision frontCamera;

    private Vision frontCamera;
    private Vision algaeCam;

    private GrabberSubsystem m_GrabberSubsystem = new GrabberSubsystem();
    private intake m_Intake = new intake(m_ElevatorSubsystem, m_ArmSubsystem, m_GrabberSubsystem);

    private ClimberSubsystem m_ClimberSubsystem = new ClimberSubsystem();

    public RobotContainer() {
        m_DriveSubsystem.registerTelemetry(logger::telemeterize);
        // VISION
        Supplier<Pose2d> PoseSupplier = () -> m_DriveSubsystem.getState().Pose;
        frontCamera = new CameraFactory().build(PoseSupplier,
                Constants.Inputs.Cameras.FrontCam);
        algaeCam = new CameraFactory().build(PoseSupplier,
                Constants.Inputs.Cameras.AlgaeCam2);
        algaeCam.changePipeline(CameraPipeline.Coral);
        // AUTON
        m_DriveSubsystem.configureAuto();
        autoDirector = new AutoDirector(new AutoSubsystems(m_DriveSubsystem, m_ArmSubsystem, m_ElevatorSubsystem,
                m_GrabberSubsystem, m_Intake,algaeCam));
        configureBindings();

    }

    // private GamePiecePhoton vision = new GamePiecePhoton();
    private void configureBindings() {
        SignalLogger.setPath("/media/sda1/logs/");
        SignalLogger.start();
        SignalLogger.enableAutoLogging(true);

        m_DriveSubsystem.setDefaultCommand(m_DriveSubsystem.Commands.applyRequest(() -> drive
                .withVelocityX(Driver.DriveLeft())
                .withVelocityY(Driver.DriveUp())
                .withRotationalRate(Driver.DriveTheta())
                .withCenterOfRotation(Driver.POV())));

        Driver.robotRel().whileTrue(m_DriveSubsystem.Commands.applyRequest(() -> driveRobot
                .withVelocityX(Driver.DriveLeft())
                .withVelocityY(Driver.DriveUp())
                .withRotationalRate(Driver.DriveTheta())
                .withCenterOfRotation(Driver.POV())));

        Driver.Seed().onTrue(m_DriveSubsystem.runOnce(() -> m_DriveSubsystem.seedFieldCentric()));
        Driver.Brake().whileTrue(m_DriveSubsystem.Commands.applyRequest(() -> brake));

        // Xbox.b().whileTrue(m_DriveSubsystem.Commands.applyRequest(() ->
        Driver.autoAlignLeft().whileTrue(
                new DeferredCommand(() -> m_DriveSubsystem.Commands.autoAlignV2("B"), Set.of(m_DriveSubsystem)));
        Driver.autoAlignRight().whileTrue(
                new DeferredCommand(() -> m_DriveSubsystem.Commands.autoAlignV2("A"), Set.of(m_DriveSubsystem)));
        Driver.outTake()
                .whileTrue(m_GrabberSubsystem.Commands.outtake().alongWith(m_ElevatorSubsystem.Commands.outtake()));
        Driver.outTake()
                .and(m_GrabberSubsystem.Commands.checkCoralReversed())
                .onTrue(Commands.runOnce(() -> SignalLogger.writeString("Scored Coral Pose", m_DriveSubsystem.getState().Pose.toString())));

        Driver.superStructureL1().onTrue(m_ElevatorSubsystem.Commands.L1().alongWith(m_ArmSubsystem.Commands.L1()));
        Driver.superStructureL2().onTrue(m_ElevatorSubsystem.Commands.L2().alongWith(m_ArmSubsystem.Commands.L2()));
        Driver.superStructureL3().onTrue(m_ElevatorSubsystem.Commands.L3().alongWith(m_ArmSubsystem.Commands.L3()));
        Driver.superStructureL4().onTrue(m_ElevatorSubsystem.Commands.L4().alongWith(m_ArmSubsystem.Commands.L4()));

        Driver.removeAlgea().and(Driver.superStructureL2())
                .onTrue(m_ElevatorSubsystem.Commands.Algea1().alongWith(m_ArmSubsystem.Commands.L1()));
        Driver.removeAlgea().and(Driver.superStructureL3())
                .onTrue(m_ElevatorSubsystem.Commands.Algea2().alongWith(m_ArmSubsystem.Commands.L1()));
        
        Driver.removeAlgea().whileTrue(m_GrabberSubsystem.Commands.outtake());

        Driver.verticalCoralIntake().whileTrue(m_GrabberSubsystem.Commands.intake()
                .alongWith(m_ArmSubsystem.Commands.GroundIntake()).alongWith(m_ElevatorSubsystem.Commands.Bottom()));
        Driver.bellyPanIntake()
                .whileTrue((m_ElevatorSubsystem.Commands.intakePrime().andThen(Commands.waitUntil(() -> m_ElevatorSubsystem.atSetpoint())))
                        .andThen(m_ArmSubsystem.Commands.Intake()));

        Driver.climberIn().onTrue(m_ClimberSubsystem.climberCommands.inwardPosition());
        Driver.climberOut().onTrue(m_ClimberSubsystem.climberCommands.outwardPosition());

        Driver.climberIn().onFalse(m_ClimberSubsystem.climberCommands.stop());
        Driver.climberOut().onFalse(m_ClimberSubsystem.climberCommands.stop());
        Driver.bellyPanIntake().and(Driver.Prime()).whileTrue(new intake(m_ElevatorSubsystem, m_ArmSubsystem, m_GrabberSubsystem));

        Driver.verticalCoralIntake().and(Driver.Prime())
                .onTrue(m_ElevatorSubsystem.Commands.Bottom().alongWith(m_ArmSubsystem.Commands.VerticalStow()));
        Driver.manUpElev().whileTrue(m_ElevatorSubsystem.Commands.changeSetpointBy(Inches.of(.5)).repeatedly());
        Driver.manDownElev().whileTrue(m_ElevatorSubsystem.Commands.changeSetpointBy(Inches.of(-.5)).repeatedly());
        Driver.manUpArm().whileTrue(m_ArmSubsystem.Commands.changeSetpointBy(Degrees.of(12)).repeatedly());
        Driver.manDownArm().whileTrue(m_ArmSubsystem.Commands.changeSetpointBy(Degrees.of(-12)).repeatedly());
        SmartDashboard.putData("Coral In the Bot", m_ElevatorSubsystem.Commands.CoralStuck().alongWith(m_ArmSubsystem.Commands.CoralStuck()));
        // SYSID ROUTINES
        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        // joystick.back().and(joystick.y()).whileTrue(m_DriveSubsystem.Commands.sysIdDynamic(Direction.kForward));
        // joystick.back().and(joystick.x()).whileTrue(m_DriveSubsystem.Commands.sysIdDynamic(Direction.kReverse));
        // joystick.start().and(joystick.y()).whileTrue(m_DriveSubsystem.Commands.sysIdQuasistatic(Direction.kForward));
        // joystick.start().and(joystick.x()).whileTrue(m_DriveSubsystem.Commands.sysIdQuasistatic(Direction.kReverse));
        // xbox.leftTrigger().whileTrue(m_Armsubsystem.Commands.runHigh());
        // xbox.rightTrigger().whileFalse(m_Armsubsystem.Commands.runRestIntake());

        // Driver.superStructureL1().onTrue(m_ElevatorSubsystem.Commands.L1().alongWith(m_ArmSubsystem.Commands.L1()));
        // Driver.superStructureL2().onTrue(m_ElevatorSubsystem.Commands.L2().alongWith(m_ArmSubsystem.Commands.L2()));
        // Driver.superStructureL3().onTrue(m_ElevatorSubsystem.Commands.L3().alongWith(m_ArmSubsystem.Commands.L3()));
        // Driver.superStructureL4().onTrue(m_ElevatorSubsystem.Commands.L4().alongWith(m_ArmSubsystem.Commands.L4()));

    }

    public void updateValues() {

        // Comment out this line if feild relitive becomes an issue.
        feedVision(frontCamera);
        feedVision(algaeCam);
        SmartDashboard.putBoolean("Has Coral", m_GrabberSubsystem.hasCoral());
        SmartDashboard.putNumber("Comms Disable Count", RobotController.getCommsDisableCount());
        SmartDashboard.putNumberArray("Speeds {X,Y,Theta}", new double[] {m_DriveSubsystem.getState().Speeds.vxMetersPerSecond,m_DriveSubsystem.getState().Speeds.vyMetersPerSecond, m_DriveSubsystem.getState().Speeds.omegaRadiansPerSecond });
    }

    public void updateSimValues() {
        // frontCamera.updateSim(m_DriveSubsystem.getState().Pose);
    }

    public Command getAutonomousCommand() {
        // return a.getSelected();
        return Commands.defer(() -> autoDirector.selection().command(),
                Set.of(m_ArmSubsystem, m_ElevatorSubsystem, m_DriveSubsystem, m_GrabberSubsystem));

    }

    public void disableLockWheels() {
        m_DriveSubsystem.Commands.applyRequest(() -> brake);
    }

    public void feedVision(Vision vision) {
        if (vision.getCameraPipeline() == CameraPipeline.AprilTag) {
            var visionEst = vision.getResults();
            if (visionEst != null) {
                visionEst.ifPresent(
                        est -> {
                            // Change our trust in the measurement based on the tags we can see
                            var estStdDevs = vision.getEstimationStdDevs();

                            m_DriveSubsystem.addVisionMeasurement(
                                    est.estimatedPose.toPose2d(), Utils.fpgaToCurrentTime(est.timestampSeconds),
                                    estStdDevs);

                            // m_DriveSubsystem.addVisionMeasurement(
                            // est.estimatedPose.toPose2d(), est.timestampSeconds);
                        });
            }
        } else {
            var visionEst = vision.getCoral();
            if(visionEst != null) {
                visionEst.ifPresent(
                    est -> {
                        logger.publishCoralPose2D(est);
                    }
                );
            }
        }
    }
}
