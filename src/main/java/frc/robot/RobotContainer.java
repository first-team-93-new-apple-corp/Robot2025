// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import java.util.function.Supplier;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.Controls.ThrottleableDrive;
import frc.robot.subsystems.Arm.ArmSubsystem;
import frc.robot.subsystems.Auton.AutoDirector;
import frc.robot.subsystems.Auton.AutoSubsystems;
import frc.robot.subsystems.Controls.ControllerSchemeIO;
import frc.robot.subsystems.Controls.POVDriveV2;
import frc.robot.subsystems.Controls.XboxDrive;
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
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second
                                                                                      // max angular velocity
    public final SwerveDriveSubsystem m_DriveSubsystem = TunerConstants.createDrivetrain();
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.02).withRotationalDeadband(MaxAngularRate * 0.02) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.RobotCentric driveRobot = new SwerveRequest.RobotCentric()
            .withDeadband(MaxSpeed * 0.02).withRotationalDeadband(MaxAngularRate * 0.02) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
    // Controls
    private final ControllerSchemeIO Driver = new XboxDrive(0);
    // Auton
    AutoDirector autoDirector;
    SendableChooser<Command> a;
    // Logging
    private final Telemetry logger = new Telemetry(MaxSpeed);
    // Arm subsystem
    private final ArmSubsystem m_Armsubsystem = new ArmSubsystem();

    private final Vision frontCamera;

    private CommandXboxController xbox;

    public RobotContainer() {
        m_DriveSubsystem.registerTelemetry(logger::telemeterize);
        // VISION
        Supplier<Pose2d> PoseSupplier = () -> m_DriveSubsystem.getState().Pose;
        frontCamera = new CameraFactory().build(PoseSupplier, Constants.Inputs.Cameras.FrontCam);
        // rearCamera = new CameraFactory().build(PoseSupplier,
        // Constants.Inputs.Cameras.RearCam);

        xbox = new CommandXboxController(0);
        // AUTON
        m_DriveSubsystem.configureAuto();
        a = AutoBuilder.buildAutoChooser();
        SmartDashboard.putData("a", a);
        SmartDashboard.putData("a", a);
        autoDirector = new AutoDirector(new AutoSubsystems(m_DriveSubsystem));
        configureBindings();

    }

    // private GamePiecePhoton vision = new GamePiecePhoton();
    private void configureBindings() {
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
        Driver.autoAlign().whileTrue(m_DriveSubsystem.Commands.autoAlign());
        // LEDS
        // Xbox.x().onTrue(LEDCommand.test(10, Color.kGreen, Color.kBlack, 25,
        // 75).andThen(LEDCommand.off()));
        // Xbox.b().onTrue(LEDCommand.shoot().andThen(LEDCommand.off()));
        // Xbox.y().onTrue(LEDCommand.test2().andThen(LEDCommand.off()));
        // Xbox.a().onTrue(getIdleLEDs());

        // SYSID ROUTINES
        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        // joystick.back().and(joystick.y()).whileTrue(m_DriveSubsystem.Commands.sysIdDynamic(Direction.kForward));
        // joystick.back().and(joystick.x()).whileTrue(m_DriveSubsystem.Commands.sysIdDynamic(Direction.kReverse));
        // joystick.start().and(joystick.y()).whileTrue(m_DriveSubsystem.Commands.sysIdQuasistatic(Direction.kForward));
        // joystick.start().and(joystick.x()).whileTrue(m_DriveSubsystem.Commands.sysIdQuasistatic(Direction.kReverse));
        // xbox.leftTrigger().whileTrue(m_Armsubsystem.Commands.runHigh());
        // xbox.rightTrigger().whileFalse(m_Armsubsystem.Commands.runRestIntake());
    }

    public void updateValues() {
        // Comment out this line if feild relitive becomes an issue.
        // feedVision(fron=tCamera);
        // feedVision(rearCamera);

    }
    public void updateSimValues() {
        frontCamera.updateSim(m_DriveSubsystem.getState().Pose);
    }
    // public void syncTime() {
    // initialTimestamp = Utils.getCurrentTimeSeconds();
    // }

    public Command getAutonomousCommand() {
        // return a.getSelected();
        return autoDirector.selection().command();
    }

    public void disableLockWheels() {
        m_DriveSubsystem.Commands.applyRequest(() -> brake);
    }

    public void feedVision(Vision vision) {
        var visionEst = vision.getResults();
        if (visionEst != null) {
            visionEst.ifPresent(
                    est -> {
                        // Change our trust in the measurement based on the tags we can see
                        var estStdDevs = vision.getEstimationStdDevs();

                        m_DriveSubsystem.addVisionMeasurement(
                                est.estimatedPose.toPose2d(), Utils.fpgaToCurrentTime(est.timestampSeconds), estStdDevs);
                                
                                
                        // m_DriveSubsystem.addVisionMeasurement(
                        // est.estimatedPose.toPose2d(), est.timestampSeconds);
                    });
        }
    }
}
