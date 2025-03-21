package frc.robot.subsystems.Auton;

import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Optional;

import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.PathPoint;
import com.pathplanner.lib.util.FlippingUtil;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.Constants.AutoConstants.IntakingStrategy;

public class PositionConstants {
    public static Rotation2d awayFromAlliance = Rotation2d.kZero;
    public static Rotation2d towardAlliance = Rotation2d.k180deg;
    public static Map<Pose2d, IntakingStrategy> intakeToStrat = Map.of(GamePeice.BlueC1, IntakingStrategy.ground,
            GamePeice.BlueC2, IntakingStrategy.ground, HumanPlayerIntake.BlueLeft, IntakingStrategy.pup,
            HumanPlayerIntake.BlueRight, IntakingStrategy.pup);

    public class GamePeice {
        public static final Pose2d BlueC1 = new Pose2d(2.300, 5.9, towardAlliance);
        public static final Pose2d BlueC2 = new Pose2d(2.300, 4.075, towardAlliance);
        public static final Pose2d BlueC3 = new Pose2d(2.300, 2.229, towardAlliance);

    }

    public class HumanPlayerIntake {
        public static final Pose2d BlueRight = new Pose2d(1.1, 0.9, Rotation2d.fromDegrees(50));
        public static final Pose2d BlueLeft = new Pose2d(1.1, 7, Rotation2d.fromDegrees(-50));
    }

    public class Reef {
        // public static final Pose2d BlueR6A = new Pose2d(3.09, 4.116, awayFromAlliance);
        // public static final Pose2d BlueR6B = new Pose2d(3.09, 3.889, awayFromAlliance);
        public static final Pose2d BlueR6A = FlippingUtil.flipFieldPose(new Pose2d(14.404, 4.225, towardAlliance));
        public static final Pose2d BlueR6B = FlippingUtil.flipFieldPose(new Pose2d(14.404, 3.784, towardAlliance));


        public static final Pose2d BlueR8A = new Pose2d(
                BlueR6A.relativeTo(new Pose2d(4.5, 4, new Rotation2d())).rotateBy(Rotation2d.fromDegrees(-60)).getX()
                        + 4.5,
                BlueR6A.relativeTo(new Pose2d(4.5, 4, new Rotation2d())).rotateBy(Rotation2d.fromDegrees(-60)).getY()
                        + 4,
                BlueR6A.relativeTo(new Pose2d(4.5, 4, new Rotation2d())).rotateBy(Rotation2d.fromDegrees(-60))
                        .getRotation());
        public static final Pose2d BlueR8B = new Pose2d(
                BlueR6B.relativeTo(new Pose2d(4.5, 4, new Rotation2d())).rotateBy(Rotation2d.fromDegrees(-60)).getX()
                        + 4.5,
                BlueR6B.relativeTo(new Pose2d(4.5, 4, new Rotation2d())).rotateBy(Rotation2d.fromDegrees(-60)).getY()
                        + 4,
                BlueR6B.relativeTo(new Pose2d(4.5, 4, new Rotation2d())).rotateBy(Rotation2d.fromDegrees(-60))
                        .getRotation());

        public static final Pose2d BlueR4A = new Pose2d(
                BlueR6A.relativeTo(new Pose2d(4.5, 4, new Rotation2d())).rotateBy(Rotation2d.fromDegrees(60)).getX()
                        + 4.5,
                BlueR6A.relativeTo(new Pose2d(4.5, 4, new Rotation2d())).rotateBy(Rotation2d.fromDegrees(60)).getY()
                        + 4,
                BlueR6A.relativeTo(new Pose2d(4.5, 4, new Rotation2d())).rotateBy(Rotation2d.fromDegrees(60))
                        .getRotation());
        public static final Pose2d BlueR4B = new Pose2d(
                BlueR6B.relativeTo(new Pose2d(4.5, 4, new Rotation2d())).rotateBy(Rotation2d.fromDegrees(60)).getX()
                        + 4.5,
                BlueR6B.relativeTo(new Pose2d(4.5, 4, new Rotation2d())).rotateBy(Rotation2d.fromDegrees(60)).getY()
                        + 4,
                BlueR6B.relativeTo(new Pose2d(4.5, 4, new Rotation2d())).rotateBy(Rotation2d.fromDegrees(60))
                        .getRotation());

        public static final Pose2d BlueR10A = new Pose2d(
                BlueR6A.relativeTo(new Pose2d(4.5, 4, new Rotation2d())).rotateBy(Rotation2d.fromDegrees(-120)).getX()
                        + 4.5,
                BlueR6A.relativeTo(new Pose2d(4.5, 4, new Rotation2d())).rotateBy(Rotation2d.fromDegrees(-120)).getY()
                        + 4,
                BlueR6A.relativeTo(new Pose2d(4.5, 4, new Rotation2d())).rotateBy(Rotation2d.fromDegrees(-120))
                        .getRotation());
        public static final Pose2d BlueR10B = new Pose2d(
                BlueR6B.relativeTo(new Pose2d(4.5, 4, new Rotation2d())).rotateBy(Rotation2d.fromDegrees(-120)).getX()
                        + 4.5,
                BlueR6B.relativeTo(new Pose2d(4.5, 4, new Rotation2d())).rotateBy(Rotation2d.fromDegrees(-120)).getY()
                        + 4,
                BlueR6B.relativeTo(new Pose2d(4.5, 4, new Rotation2d())).rotateBy(Rotation2d.fromDegrees(-120))
                        .getRotation());

        public static final Pose2d BlueR12A = new Pose2d(
                BlueR6A.relativeTo(new Pose2d(4.5, 4, new Rotation2d())).rotateBy(Rotation2d.fromDegrees(180)).getX()
                        + 4.5,
                BlueR6A.relativeTo(new Pose2d(4.5, 4, new Rotation2d())).rotateBy(Rotation2d.fromDegrees(180)).getY()
                        + 4,
                BlueR6A.relativeTo(new Pose2d(4.5, 4, new Rotation2d())).rotateBy(Rotation2d.fromDegrees(180))
                        .getRotation());
        public static final Pose2d BlueR12B = new Pose2d(
                BlueR6B.relativeTo(new Pose2d(4.5, 4, new Rotation2d())).rotateBy(Rotation2d.fromDegrees(180)).getX()
                        + 4.5,
                BlueR6B.relativeTo(new Pose2d(4.5, 4, new Rotation2d())).rotateBy(Rotation2d.fromDegrees(180)).getY()
                        + 4,
                BlueR6B.relativeTo(new Pose2d(4.5, 4, new Rotation2d())).rotateBy(Rotation2d.fromDegrees(180))
                        .getRotation());

        public static final Pose2d BlueR2A = new Pose2d(
                BlueR6A.relativeTo(new Pose2d(4.5, 4, new Rotation2d())).rotateBy(Rotation2d.fromDegrees(120)).getX()
                        + 4.5,
                BlueR6A.relativeTo(new Pose2d(4.5, 4, new Rotation2d())).rotateBy(Rotation2d.fromDegrees(120)).getY()
                        + 4,
                BlueR6A.relativeTo(new Pose2d(4.5, 4, new Rotation2d())).rotateBy(Rotation2d.fromDegrees(120))
                        .getRotation());
        public static final Pose2d BlueR2B = new Pose2d(
                BlueR6B.relativeTo(new Pose2d(4.5, 4, new Rotation2d())).rotateBy(Rotation2d.fromDegrees(120)).getX()
                        + 4.5,
                BlueR6B.relativeTo(new Pose2d(4.5, 4, new Rotation2d())).rotateBy(Rotation2d.fromDegrees(120)).getY()
                        + 4,
                BlueR6B.relativeTo(new Pose2d(4.5, 4, new Rotation2d())).rotateBy(Rotation2d.fromDegrees(120))
                        .getRotation());
        public static final List<Pose2d> BlueReef = List.of(BlueR2A, BlueR2B, BlueR4A, BlueR4B, BlueR6A, BlueR6B,
                BlueR8A, BlueR8B, BlueR10A, BlueR10B, BlueR12A, BlueR12B);
        public static final List<Pose2d> BlueReefA = List.of(BlueR2A, BlueR4A, BlueR6A, BlueR8A, BlueR10A, BlueR12A);
        public static final List<Pose2d> BlueReefB = List.of(BlueR2B, BlueR4B, BlueR6B, BlueR8B, BlueR10B, BlueR12B);

        public static final List<Pose2d> RedReef = BlueReef.stream().map(FlippingUtil::flipFieldPose).toList();
        public static final List<Pose2d> RedReefA = BlueReefA.stream().map(FlippingUtil::flipFieldPose).toList();
        public static final List<Pose2d> RedReefB = BlueReefB.stream().map(FlippingUtil::flipFieldPose).toList();

    }

    public static Pose2d poseFromPathEnd(PathPlannerPath path) {
        PathPoint point = path.getAllPathPoints().get(path.getAllPathPoints().size() - 1);

        return new Pose2d(point.position, point.rotationTarget.rotation());
    }

    public class startingPoses {

        private static final double startingLineBlue = 7.2;

        public static final Pose2d leftBlue = new Pose2d(startingLineBlue, 7.05, towardAlliance);
        public static final Pose2d rightBlue = new Pose2d(startingLineBlue, 0.475, towardAlliance);
        public static final Pose2d CenterCoralBlue = new Pose2d(1.232, 4.033, awayFromAlliance);
        public static final Pose2d CenterCoralRed = FlippingUtil.flipFieldPose(CenterCoralBlue);
        public static final Pose2d CenterLeftBlue = new Pose2d(1.232, 4.933, awayFromAlliance);
        public static final Pose2d CenterLeftRed = FlippingUtil.flipFieldPose(CenterLeftBlue);
        public static final Pose2d leftRed = FlippingUtil.flipFieldPose(leftBlue);
        public static final Pose2d rightRed = FlippingUtil.flipFieldPose(rightBlue);
        public static final Pose2d Center = new Pose2d(7.2, 4, towardAlliance);

        public static final Pose2d AllianceCorrectedPose(Pose2d pose) {
            Optional<Alliance> ally = DriverStation.getAlliance();

            if (ally.isPresent()) {
                if (ally.get() == Alliance.Red) {
                    return FlippingUtil.flipFieldPose(pose);
                }
                if (ally.get() == Alliance.Blue) {
                    return pose;
                } else {
                    return pose;
                }
            } else {
                return pose;
            }
        }

        public static final Pose2d Left() {
            Optional<Alliance> ally = DriverStation.getAlliance();
            if (ally.isPresent()) {
                if (ally.get() == Alliance.Red) {
                    return leftRed;
                }
                if (ally.get() == Alliance.Blue) {
                    return leftBlue;
                } else {
                    return leftBlue;
                }
            } else {
                return leftBlue;
            }
        }

        public static final Pose2d Center() {
            return AllianceCorrectedPose(Center);
        }
        public static final Pose2d RightMidBarge() {
            return AllianceCorrectedPose(new Pose2d(7.1,1.85, towardAlliance));
        }

        public static final Pose2d LeftBargeMiddle() {
            return AllianceCorrectedPose(new Pose2d(7.2, 6.2, towardAlliance));
        }

        public static final Pose2d CeneterCoral() {
            Optional<Alliance> ally = DriverStation.getAlliance();
            if (ally.isPresent()) {
                if (ally.get() == Alliance.Red) {
                    return CenterCoralRed;
                }
                if (ally.get() == Alliance.Blue) {
                    return CenterCoralBlue;
                } else {
                    return CenterCoralBlue;
                }
            } else {
                return CenterCoralBlue;
            }
        }

        public static final Pose2d CenterLeft() {
            Optional<Alliance> ally = DriverStation.getAlliance();
            if (ally.isPresent()) {
                if (ally.get() == Alliance.Red) {
                    return CenterLeftRed;
                }
                if (ally.get() == Alliance.Blue) {
                    return CenterLeftBlue;
                } else {
                    return CenterLeftBlue;
                }
            } else {
                return CenterLeftBlue;
            }
        }

        public static final Pose2d right() {
            Optional<Alliance> ally = DriverStation.getAlliance();
            if (ally.isPresent()) {
                if (ally.get() == Alliance.Red) {
                    return rightRed;
                }
                if (ally.get() == Alliance.Blue) {
                    return rightBlue;
                } else {
                    return rightBlue;
                }
            } else {
                return rightBlue;
            }
        }

    }
}
