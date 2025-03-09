package frc.robot.subsystems.Auton;

import java.util.Optional;

import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.PathPoint;
import com.pathplanner.lib.util.FlippingUtil;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public class PositionConstants {
    private static Rotation2d awayFromAlliance = Rotation2d.kZero;
    private static Rotation2d towardAlliance = Rotation2d.k180deg;

    public class GamePeice {
  
    }
    public class Reef {
        public static final Pose2d BlueR6A = new Pose2d(3.220,  3.840, awayFromAlliance);
        public static final Pose2d BlueR6B = new Pose2d(3.275,  4.165, awayFromAlliance);

    }

    public class startingPoses {

        public static final Pose2d AllianceCorrectedPose(Pose2d pose) {
            Optional<Alliance> ally = DriverStation.getAlliance();
            if (ally.isPresent()) {
                if (ally.get() == Alliance.Red) {
                    return FlippingUtil.flipFieldPose(pose);
                }
            }
            return pose;
        }
        
        private static final double startingLineBlue = 7.2;

        public static final Pose2d leftBlue = new Pose2d(startingLineBlue, 7.05, towardAlliance);
        public static final Pose2d rightBlue = new Pose2d(startingLineBlue, 0.475, towardAlliance); 
        public static final Pose2d CenterCoralBlue = new Pose2d(1.232, 4.033, awayFromAlliance); 
        public static final Pose2d CenterCoralRed = FlippingUtil.flipFieldPose(CenterCoralBlue);
        public static final Pose2d CenterLeftBlue = new Pose2d(1.232, 4.933, awayFromAlliance); 
        public static final Pose2d CenterLeftRed = FlippingUtil.flipFieldPose(CenterLeftBlue);
        public static final Pose2d leftRed = FlippingUtil.flipFieldPose(leftBlue);
        public static final Pose2d rightRed = FlippingUtil.flipFieldPose(rightBlue);
        
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

        public static Pose2d poseFromPathEnd(PathPlannerPath path){
            PathPoint point = path.getAllPathPoints().get(path.getAllPathPoints().size()-1);
            return new Pose2d(point.position, point.rotationTarget.rotation());
        }
    }
}
