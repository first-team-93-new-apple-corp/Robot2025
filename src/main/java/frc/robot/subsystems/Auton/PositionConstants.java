package frc.robot.subsystems.Auton;

import java.util.Optional;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.Utilities.AllianceFlipUtil;

public class PositionConstants {
    private static Rotation2d awayFromAlliance = new Rotation2d();
    private static Rotation2d towardAlliance = new Rotation2d().rotateBy(Rotation2d.k180deg);

    public class GamePeice {
  
    }

    public class startingPoses {
        private static final double startingLineBlue = 2.9;

        public static final Pose2d leftBlue = new Pose2d(startingLineBlue, 7.624, towardAlliance);
        public static final Pose2d rightBlue = new Pose2d(startingLineBlue, .475, towardAlliance); 
        public static final Pose2d leftRed = AllianceFlipUtil.flip(leftBlue);
        public static final Pose2d rightRed = AllianceFlipUtil.flip(rightBlue);

        public static final Pose2d top() {
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
