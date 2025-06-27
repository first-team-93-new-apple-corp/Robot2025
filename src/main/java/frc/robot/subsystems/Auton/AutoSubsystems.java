package frc.robot.subsystems.Auton;

import frc.robot.commands.intake;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.Arm.ArmSubsystem;
import frc.robot.subsystems.Grabber.GrabberSubsystem;
import frc.robot.subsystems.Intake.IntakeSubsystem;
import frc.robot.subsystems.Swerve.SwerveDriveSubsystem;
import frc.robot.subsystems.VisionIO.Vision;

public record AutoSubsystems(
    SwerveDriveSubsystem driveSubsystem,
    ArmSubsystem armSubsystem,
    ElevatorSubsystem elevatorSubsystem,
    GrabberSubsystem grabberSubsystem,
    intake intakeCommand,
    Vision cam,
    IntakeSubsystem intakeSubsystem

) {}