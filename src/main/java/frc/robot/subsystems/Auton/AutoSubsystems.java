package frc.robot.subsystems.Auton;

import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.Arm.ArmSubsystem;
import frc.robot.subsystems.Grabber.GrabberSubsystem;
import frc.robot.subsystems.Swerve.SwerveDriveSubsystem;

public record AutoSubsystems(
    SwerveDriveSubsystem driveSubsystem,
    ArmSubsystem armSubsystem,
    ElevatorSubsystem elevatorSubsystem,
    GrabberSubsystem grabberSubsystem
) {}