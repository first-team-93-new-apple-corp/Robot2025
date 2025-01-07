package frc.robot.subsystems.Auton;

import java.util.List;
import java.util.function.Supplier;
import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Swerve.TunerConstants;
import frc.robot.subsystems.VisionIO.Vision;

public class AutoTracker {
    SequentialCommandGroup commands = new SequentialCommandGroup();
    PathPlannerPath intakingpath;
    PathPlannerPath Shootingpath;
    PathConstraints constraints = new PathConstraints(TunerConstants.kSpeedAt12Volts.in(MetersPerSecond), 10.2, 9, 30);
    // Vision m_Vision = new Vision();

    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond) / 1.5; // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.02).withRotationalDeadband(MaxAngularRate * 0.02)
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    public AutoTracker( AutoSubsystems subsystems, List<AutoSector> paths, Supplier<Pose2d> initalPose){
        commands.addCommands(Commands.runOnce(() -> subsystems.driveSubsystem().resetPose(initalPose.get()), subsystems.driveSubsystem()));
        for (AutoSector autoSector : paths) {
            try {
                intakingpath = PathPlannerPath.fromPathFile(autoSector.intakingPath());
                Shootingpath = PathPlannerPath.fromPathFile(autoSector.ShootingPath());
                commands.addCommands(AutoBuilder.followPath(intakingpath));
                commands.addCommands(Commands.print("Vision Note Grab"));
                // commands.addCommands(subsystems.driveSubsystem().Commands.applyRequest(() -> drive.withRotationalRate(2)).withTimeout(Math.PI));
                // commands.addCommands(subsystems.driveSubsystem().Commands.applyRequest(() -> drive.withRotationalRate(m_Vision.turnToNote())));
                commands.addCommands(AutoBuilder.pathfindThenFollowPath(Shootingpath, constraints));
                commands.addCommands(Commands.print("Bang Bang (shot the note)"));
            } catch (Exception e) {}
        }
        try {
            commands.addCommands(AutoBuilder.followPath(PathPlannerPath.fromPathFile("Leave")));
        } catch (Exception e) {}
    }
    public Command asCommand(){
        return commands;
    }
}
