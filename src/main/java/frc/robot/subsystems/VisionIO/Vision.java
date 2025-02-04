package frc.robot.subsystems.VisionIO;

import java.io.PrintWriter;
import java.util.List;
import java.util.Optional;
import java.util.function.Supplier;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Vision extends SubsystemBase {
    Supplier<Pose2d> poseSupplier;
    private PhotonCamera Camera;
    private PhotonPoseEstimator PoseEstimator;
    private Matrix<N3, N1> curStdDevs;
    private Matrix<N3, N1> kSingleTagStdDevs = VecBuilder.fill(4, 4, 8);
    private Matrix<N3, N1> kMultiTagStdDevs = VecBuilder.fill(0.5, 0.5, 1);

    public record VisionResults(Pose2d pose, double Time, Matrix<N3, N1> StdDevs) {
    };

    public Vision(Supplier<Pose2d> PoseSupplier, String camName, Transform3d camTransform) {
        poseSupplier = PoseSupplier;
        Camera = new PhotonCamera(camName);
        PoseEstimator = new PhotonPoseEstimator(AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape),
                PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, camTransform);
        PoseEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);

    }

    private void updateEstimationStdDevs(
            Optional<EstimatedRobotPose> estimatedPose, List<PhotonTrackedTarget> targets,
            PhotonPoseEstimator photonEstimator) {
        if (estimatedPose.isEmpty()) {
            // No pose input. Default to single-tag std devs
            curStdDevs = kSingleTagStdDevs;

        } else {
            // Pose present. Start running Heuristic
            Matrix<N3, N1> estStdDevs = kSingleTagStdDevs;
            int numTags = 0;
            double avgDist = 0;
            
            // Precalculation - see how many tags we found, and calculate an
            // average-distance metric
            for (PhotonTrackedTarget tgt : targets) {
                var tagPose = photonEstimator.getFieldTags().getTagPose(tgt.getFiducialId());
                if (tagPose.isEmpty())
                    continue;
                numTags++;
                avgDist += tagPose
                        .get()
                        .toPose2d()
                        .getTranslation()
                        .getDistance(estimatedPose.get().estimatedPose.toPose2d().getTranslation());
            }

            if (numTags == 0) {
                // No tags visible. Default to single-tag std devs
                curStdDevs = kSingleTagStdDevs;
            } else {
                // One or more tags visible, run the full heuristic.
                avgDist /= numTags;
                // Decrease std devs if multiple targets are visible
                if (numTags > 1)
                    estStdDevs = kMultiTagStdDevs;
                // Increase std devs based on (average) distance
                if (numTags == 1 && avgDist > 4)
                    estStdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
                else
                    estStdDevs = estStdDevs.times(1 + (avgDist * avgDist / 30));
                curStdDevs = estStdDevs;
            }
        }
    }

    public Matrix<N3, N1> getEstimationStdDevs() {
        return curStdDevs;
    }

    public Optional<EstimatedRobotPose> getEstimatedGlobalPose(PhotonCamera camera,
            PhotonPoseEstimator photonEstimator) {
        Optional<EstimatedRobotPose> visionEst = Optional.empty();
        for (var change : camera.getAllUnreadResults()) {
            visionEst = photonEstimator.update(change);
            updateEstimationStdDevs(visionEst, change.getTargets(), photonEstimator);
        }
        if (visionEst.isEmpty()) {
            visionEst = null;
        }
        return visionEst;
    }

    public Optional<EstimatedRobotPose> getResults() {
        return getEstimatedGlobalPose(Camera, PoseEstimator);
    }

    // public Optional<EstimatedRobotPose> getEstimatedGlobalPoseFront(Pose2d
    // prevEstimatedRobotPose) {
    // frontPoseEstimator.setReferencePose(prevEstimatedRobotPose);
    // return frontPoseEstimator.update();
    // }
    // public Optional<EstimatedRobotPose> getEstimatedGlobalPoseRear(Pose2d
    // prevEstimatedRobotPose) {
    // rearPoseEstimator.setReferencePose(prevEstimatedRobotPose);
    // return rearPoseEstimator.update();
    // }

}
