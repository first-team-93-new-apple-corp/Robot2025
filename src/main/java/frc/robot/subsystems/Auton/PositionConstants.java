package frc.robot.subsystems.Auton;

import java.util.Optional;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.Utilities.AllianceFlipUtil;

public class PositionConstants {
    private static Rotation2d NoRotation = new Rotation2d();

    public class GamePeice {
        public static final double SGBlueX = 2.9;
        public static final double CGX = 8.29;

        public static final Pose2d SG1Blue = new Pose2d(SGBlueX, 7, NoRotation);
        public static final Pose2d SG2Blue = new Pose2d(SGBlueX, 5.560, NoRotation);
        public static final Pose2d SG3Blue = new Pose2d(SGBlueX, 4.114, NoRotation);
        public static final Pose2d SG1Red = AllianceFlipUtil.flip(SG1Blue);
        public static final Pose2d SG2Red = AllianceFlipUtil.flip(SG2Blue);
        public static final Pose2d SG3Red = AllianceFlipUtil.flip(SG3Blue);

        public static final Pose2d CG1 = new Pose2d(CGX, 7.43, NoRotation);
        public static final Pose2d CG2 = new Pose2d(CGX, 5.775, NoRotation);
        public static final Pose2d CG3 = new Pose2d(CGX, 4.1, NoRotation);
        public static final Pose2d CG4 = new Pose2d(CGX, 2.47, NoRotation);
        public static final Pose2d CG5 = new Pose2d(CGX, 0.765, NoRotation);
    }

    public static final Pose2d BlueSpeakerCenter = new Pose2d(1.3200, 5.560, NoRotation);
    public static final Pose2d RedSpeakerCenter = AllianceFlipUtil.flip(BlueSpeakerCenter);

    public static final Pose2d Speaker() {
        Optional<Alliance> ally = DriverStation.getAlliance();
        if (ally.isPresent()) {
            if (ally.get() == Alliance.Red) {
                return RedSpeakerCenter;
            }
            if (ally.get() == Alliance.Blue) {
                return BlueSpeakerCenter;
            } else {
                return BlueSpeakerCenter;
            }
        } else {
            return BlueSpeakerCenter;
        }
    }
}
