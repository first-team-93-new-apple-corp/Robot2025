package frc.robot.subsystems.Auton;

import java.io.IOException;
import java.util.List;
import java.util.Set;
import java.util.function.Supplier;

import org.json.simple.parser.ParseException;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.FileVersionException;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.AutoConstants.AutoSector;
import frc.robot.Constants.AutoConstants.AutoSectorV2;
import frc.robot.Constants.AutoConstants.IntakingStrategy;
import frc.robot.commands.intake;
import frc.robot.subsystems.Arm.ArmSubsystem;
import frc.robot.subsystems.Auton.AutoDirector.Auto;

public class AutoTrackerV2 extends SequentialCommandGroup {

    private AutoSubsystems subsystems;

    private PathConstraints constraints = AutoConstants.constraints;

    public AutoTrackerV2(AutoSubsystems subsystems, Supplier<Pose2d> initalPose) {
        SmartDashboard.putBoolean("scored", false);
        this.subsystems = subsystems;
        addCommands(Commands.print("Auto Time"));
        addCommands(Commands.runOnce(() -> subsystems.driveSubsystem().resetPose(initalPose.get()),
                subsystems.driveSubsystem()));

    }

    public void addPreload(String preload) {
        try {
            PathPlannerPath path = PathPlannerPath.fromPathFile(preload);
            addCommands(score(path));
        } catch (Exception e) {
        }
    }

    public void addPreloadV2(String preload) {
        try {
            PathPlannerPath path = PathPlannerPath.fromPathFile(preload);

            addPreload(PositionConstants.poseFromPathEnd(path));
        } catch (Exception e) {
        }
    }

    public void addPreload(Pose2d preload) {
        try {

            addCommands(score(preload));
        } catch (Exception e) {
        }
    }

    public void addPreloadAdvanced(Pose2d preload) {
        try {

            addCommands(advancedScore(PositionConstants.Reef.poseToPreAlign(preload), preload));
        } catch (Exception e) {
        }
    }

    public void addSector(AutoSector sector) {
        try {
            PathPlannerPath intakingpath = PathPlannerPath.fromPathFile(sector.intakingPath());
            PathPlannerPath scoringPath = PathPlannerPath.fromPathFile(sector.ShootingPath());

            addCommands(AutoBuilder.pathfindThenFollowPath(intakingpath, constraints));
            addCommands(new ConditionalCommand(
                    subsystems.elevatorSubsystem().Commands.Bottom()
                            .alongWith(subsystems.armSubsystem().Commands.GroundIntake()
                                    .alongWith(Commands.waitSeconds(.5))),
                    Commands.print("Yeah were there"), () -> subsystems.elevatorSubsystem().atBottom()));
            addCommands(Intake(IntakingStrategy.ground));

            addCommands((subsystems.driveSubsystem().Commands
                    .applyRequest(() -> new SwerveRequest.RobotCentric().withVelocityX(-.5))
                    .withDeadline(Commands.waitSeconds(.75))).andThen(subsystems.driveSubsystem().Commands
                            .applyRequest(() -> new SwerveRequest.RobotCentric().withVelocityX(0)))
                    .withDeadline(Commands.waitSeconds(1)));

            addCommands(score(scoringPath));

        } catch (Exception e) {
            e.printStackTrace();

        }
    }

    public void addSectorV2(AutoSector sector) {
        try {
            PathPlannerPath intakingpath = PathPlannerPath.fromPathFile(sector.intakingPath());
            PathPlannerPath scoringPath = PathPlannerPath.fromPathFile(sector.ShootingPath());

            addSector(new AutoSectorV2(PositionConstants.poseFromPathEnd(intakingpath),
                    PositionConstants.poseFromPathEnd(scoringPath)));
        } catch (Exception e) {
            e.printStackTrace();

        }
    }

    public Pose2d checkIfCoral(Pose2d suggestion, Pose2d constant) {
        Transform2d a = suggestion.minus(constant);
        double Distance = Math.sqrt((Math.pow(a.getX(), 2))+(Math.pow(a.getY(), 2)));
        if (Distance < 2) {
            return suggestion;
        }
        return constant;
    }

    public void addPoint(Pose2d point) {
        addCommands(AutoBuilder.pathfindToPoseFlipped(point, constraints));
    }

    public void addSector(AutoSectorV2 sector) {
        try {
            addCommands(
                    Commands.defer(
                            () -> AutoBuilder.pathfindToPoseFlipped(checkIfCoral(subsystems.cam().Coral, sector.intakingPose()), constraints),
                            Set.of(subsystems.driveSubsystem())));
            addCommands(
                    new ConditionalCommand(
                            subsystems.elevatorSubsystem().Commands.Bottom()
                                    .alongWith(subsystems.armSubsystem().Commands.GroundIntake()
                                            .alongWith(Commands.waitSeconds(.5))),
                            Commands.print("Yeah were there"), () -> subsystems.elevatorSubsystem().atBottom()));
            addCommands(Intake(PositionConstants.intakeToStrat.get(sector.intakingPose())));

            addCommands((subsystems.driveSubsystem().Commands
                    .applyRequest(() -> new SwerveRequest.RobotCentric().withVelocityX(-.5))
                    .withDeadline(Commands.waitSeconds(.75))).andThen(subsystems.driveSubsystem().Commands
                            .applyRequest(() -> new SwerveRequest.RobotCentric().withVelocityX(0)))
                    .withDeadline(Commands.waitSeconds(1)));

            addCommands(score(sector.ScoringPose()));

        } catch (Exception e) {
            e.printStackTrace();

        }
    }

    public void addSectorAdvanced(AutoSectorV2 sector) {
        try {
            addCommands(
                    Commands.defer(
                            () -> AutoBuilder.pathfindToPoseFlipped(checkIfCoral(subsystems.cam().Coral, sector.intakingPose()), constraints),
                            Set.of(subsystems.driveSubsystem())));
            addCommands(
                    new ConditionalCommand(
                            subsystems.elevatorSubsystem().Commands.Bottom()
                                    .alongWith(subsystems.armSubsystem().Commands.GroundIntake()
                                            .alongWith(Commands.waitSeconds(.5))),
                            Commands.print("Yeah were there"), () -> subsystems.elevatorSubsystem().atBottom()));
            addCommands(Intake(PositionConstants.intakeToStrat.get(sector.intakingPose())));

            addCommands((subsystems.driveSubsystem().Commands
                    .applyRequest(() -> new SwerveRequest.RobotCentric().withVelocityX(-.5))
                    .withDeadline(Commands.waitSeconds(.75))).andThen(subsystems.driveSubsystem().Commands
                            .applyRequest(() -> new SwerveRequest.RobotCentric().withVelocityX(0)))
                    .withDeadline(Commands.waitSeconds(1)));

            addCommands(advancedScore(PositionConstants.Reef.poseToPreAlign(sector.ScoringPose()), sector.ScoringPose()));

        } catch (Exception e) {
            e.printStackTrace();

        }
    }

    public Command score(PathPlannerPath scoringPath) {
        return score(AutoBuilder.pathfindThenFollowPath(scoringPath, constraints));
    }

    public Command score(Pose2d scoringPose) {

        return (new ConditionalCommand(Commands.runOnce(() -> SmartDashboard.putBoolean("scored", true)),
                ((AutoBuilder.pathfindToPoseFlipped(scoringPose, constraints).alongWith(L4())
                        .alongWith(Commands.print("Going there"))))
                        .andThen(Commands.print("Got there"))
                        .andThen(Outtake())
                        .andThen((subsystems.driveSubsystem().Commands
                                .applyRequest(() -> new SwerveRequest.RobotCentric().withVelocityX(-1))
                                .withDeadline(Commands.waitSeconds(.7)))
                                .andThen(subsystems.driveSubsystem().Commands
                                        .applyRequest(() -> new SwerveRequest.RobotCentric().withVelocityX(0)))
                                .withDeadline(Commands.waitSeconds(1))),
                () -> !SmartDashboard.getBoolean("Has Coral", false))
                .repeatedly().until((() -> SmartDashboard.getBoolean("scored", false))))
                .andThen(Commands.runOnce(() -> SmartDashboard.putBoolean("scored", false)))
                .andThen((subsystems.driveSubsystem().Commands
                        .applyRequest(() -> new SwerveRequest.RobotCentric().withVelocityX(1))
                        .withDeadline(Commands.waitSeconds(.4)))
                        .andThen(subsystems.driveSubsystem().Commands
                                .applyRequest(() -> new SwerveRequest.RobotCentric().withVelocityX(0)))
                        .withDeadline(Commands.waitSeconds(.7)));
    }

    public Command score(Command pathFollowing) {
        return (new ConditionalCommand(Commands.runOnce(() -> SmartDashboard.putBoolean("scored", true)),
                ((pathFollowing.alongWith(L4()).alongWith(Commands.print("Going there"))))
                        .andThen(Commands.print("Got there"))
                        .andThen(Outtake())
                        .andThen((subsystems.driveSubsystem().Commands
                                .applyRequest(() -> new SwerveRequest.RobotCentric().withVelocityX(-1))
                                .withDeadline(Commands.waitSeconds(.6)))
                                .andThen(subsystems.driveSubsystem().Commands
                                        .applyRequest(() -> new SwerveRequest.RobotCentric().withVelocityX(0)))
                                .withDeadline(Commands.waitSeconds(1))),
                () -> !SmartDashboard.getBoolean("Has Coral", false))
                .repeatedly().until((() -> SmartDashboard.getBoolean("scored", false))))
                .andThen(Commands.runOnce(() -> SmartDashboard.putBoolean("scored", false)))
                .andThen((subsystems.driveSubsystem().Commands
                        .applyRequest(() -> new SwerveRequest.RobotCentric().withVelocityX(1))
                        .withDeadline(Commands.waitSeconds(.4)))
                        .andThen(subsystems.driveSubsystem().Commands
                                .applyRequest(() -> new SwerveRequest.RobotCentric().withVelocityX(0)))
                        .withDeadline(Commands.waitSeconds(.7)));
    }

    public Command advancedScore(Pose2d preScoringPose, Pose2d scoringPose) {
        return new ConditionalCommand(Commands.runOnce(() -> SmartDashboard.putBoolean("scored", true)),
                (Commands.defer(() -> AutoBuilder.pathfindToPoseFlipped(preScoringPose, AutoConstants.getHeightAdjustedConstraints(subsystems.elevatorSubsystem().armPivotHeight())), Set.of(subsystems.driveSubsystem())))
                        .alongWith(Commands.print("Going there"))
                        .andThen(L4())
                        .andThen(AutoBuilder.pathfindToPoseFlipped(scoringPose, constraints))
                        .andThen(Outtake())
                        .andThen((subsystems.driveSubsystem().Commands
                                .applyRequest(() -> new SwerveRequest.RobotCentric().withVelocityX(-1))
                                .withDeadline(Commands.waitSeconds(.6)))
                                .andThen(subsystems.driveSubsystem().Commands
                                        .applyRequest(() -> new SwerveRequest.RobotCentric().withVelocityX(0))
                                .withDeadline(Commands.waitSeconds(1)))),
                () -> !SmartDashboard.getBoolean("Has Coral", false))
                .repeatedly().until((() -> SmartDashboard.getBoolean("scored", false)))
                .andThen(Commands.runOnce(() -> SmartDashboard.putBoolean("scored", false)))
                .andThen(((subsystems.driveSubsystem().Commands
                        .applyRequest(() -> new SwerveRequest.RobotCentric().withVelocityX(1))
                        .withDeadline(Commands.waitSeconds(.4)))
                        .andThen(subsystems.driveSubsystem().Commands
                                .applyRequest(() -> new SwerveRequest.RobotCentric().withVelocityX(0)))
                        .withDeadline(Commands.waitSeconds(.6)))
                .alongWith((subsystems.armSubsystem().Commands.VerticalStow())
                .alongWith(Commands.waitUntil(() -> subsystems.armSubsystem().atSetpoint()))
                .andThen(subsystems.elevatorSubsystem().Commands.Bottom())));
    }

    private Command L2() {
        if (Utils.isSimulation()) {
            return Commands.print("to L4");
        } else {
            return subsystems.armSubsystem().Commands.L2().alongWith(subsystems.elevatorSubsystem().Commands.L2());
        }
    }

    private Command L4() {
        if (Utils.isSimulation()) {
            return Commands.print("to L4");
        } else {
            return subsystems.armSubsystem().Commands.L4().alongWith(subsystems.elevatorSubsystem().Commands.L4()).alongWith(Commands.waitSeconds(1));
        }
    }

    private Command L1() {
        if (Utils.isSimulation()) {
            return Commands.print("to L1");
        } else {

            return subsystems.armSubsystem().Commands.L1().alongWith(subsystems.elevatorSubsystem().Commands.L1());
        }
    }

    private Command Intake(IntakingStrategy strategy) {
        if (Utils.isSimulation()) {
            return Commands.print("Intake");

        } else {
            // ground
            if (strategy.equals(IntakingStrategy.pup)) {
                // subsystems.driveSubsystem().Commands
                // .applyRequest(() -> new SwerveRequest.RobotCentric().withVelocityX(-.5))
                // .withDeadline(Commands.waitSeconds(0.1)))
                // .andThen(subsystems.driveSubsystem().Commands
                // .applyRequest(() -> new SwerveRequest.RobotCentric().withVelocityX(0))))
                // .withDeadline(Commands.waitSeconds(0.15))
                // .andThen
                return subsystems.elevatorSubsystem().Commands.intake().andThen(Commands.waitSeconds(0.3))
                        .andThen((subsystems.armSubsystem().Commands.Intake())).andThen(Commands.waitSeconds(2))
                        .andThen(new intake(subsystems.elevatorSubsystem(), subsystems.armSubsystem(),
                                subsystems.grabberSubsystem()));

                // cmd.addCommands(subsystems.driveSubsystem().Commands.applyRequest(
                // () -> new SwerveRequest.RobotCentric()
                // .withVelocityX(5)));
                // cmd.addCommands(Commands.waitSeconds(0.1));
                // cmd.addCommands(subsystems.driveSubsystem().Commands
                // .applyRequest(() -> new SwerveRequest.RobotCentric().withVelocityX(0)));
                // .andThen(subsystems.intakeCommand())
                // .andThen(subsystems.elevatorSubsystem().Commands.L2().alongWith(subsystems.armSubsystem().Commands.GroundIntake()));

            }
            return ((subsystems.grabberSubsystem().Commands.intake()
                    .alongWith(subsystems.armSubsystem().Commands.GroundIntake())
                    .alongWith(subsystems.elevatorSubsystem().Commands.Bottom()))
                    .alongWith((subsystems.driveSubsystem().Commands
                            .applyRequest(() -> new SwerveRequest.RobotCentric().withVelocityX(.5))
                            .withDeadline(Commands.waitUntil(() -> SmartDashboard.getBoolean("Has Coral", true))))
                            .andThen(subsystems.driveSubsystem().Commands
                                    .applyRequest(() -> new SwerveRequest.RobotCentric().withVelocityX(0))))
                    .withDeadline(Commands.waitSeconds(1.6)))
                    .andThen((subsystems.driveSubsystem().Commands
                            .applyRequest(() -> new SwerveRequest.RobotCentric().withVelocityX(-.5))
                            .withDeadline(Commands.waitSeconds(.5))).andThen(subsystems.driveSubsystem().Commands
                                    .applyRequest(() -> new SwerveRequest.RobotCentric().withVelocityX(0)))
                            .withDeadline(Commands.waitSeconds(.7)));
        }
    }

    private Command Outtake() {
        if (Utils.isSimulation()) {
            return Commands.print("outtake");

        } else {
            return (subsystems.elevatorSubsystem().Commands.outtake().alongWith(Commands.waitSeconds(1)))
                    .alongWith(subsystems.grabberSubsystem().Commands.outtake().withDeadline(Commands.waitSeconds(1)));
        }
    }
}
