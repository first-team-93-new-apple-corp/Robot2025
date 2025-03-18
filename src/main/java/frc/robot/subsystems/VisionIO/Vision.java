package frc.robot.subsystems.VisionIO;

import static edu.wpi.first.units.Units.Inches;

import java.util.List;
import java.util.Optional;
import java.util.function.Supplier;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.estimation.TargetModel;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Inputs.CameraPipeline;
import frc.robot.Robot;

public class Vision extends SubsystemBase {
    Supplier<Pose2d> poseSupplier;
    private PhotonCamera Camera;
    private String camName;
    private Transform3d camTransform;
    // Sim
    private VisionSystemSim systemSim;
    private AprilTagFieldLayout simTagLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded);
    private SimCameraProperties simCameraProperties;
    private PhotonCameraSim cameraSim;
    // End Sim
    private PhotonPoseEstimator PoseEstimator;
    private Matrix<N3, N1> curStdDevs;
    private Matrix<N3, N1> kSingleTagStdDevs = VecBuilder.fill(4, 4, 8);
    private Matrix<N3, N1> kMultiTagStdDevs = VecBuilder.fill(0.5, 0.5, 1);
    private CameraPipeline currentPipeline;

    public record VisionResults(Pose2d pose, double Time, Matrix<N3, N1> StdDevs) {
    };

    public Vision(Supplier<Pose2d> PoseSupplier, String camName, Transform3d camTransform) {
        this.camName = camName;
        this.camTransform = camTransform;
        poseSupplier = PoseSupplier;
        Camera = new PhotonCamera(camName);
        PoseEstimator = new PhotonPoseEstimator(AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded),
                PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, camTransform);
        PoseEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
        changePipeline(CameraPipeline.AprilTag);
        // Sim
        if (Robot.isSimulation()) {
            defineSim();
        }
    }

    public void defineSim() {
        systemSim = new VisionSystemSim(camName + "Sim");
        systemSim.addAprilTags(simTagLayout);
        simCameraProperties = new SimCameraProperties();
        simCameraProperties.setCalibration(1280, 720, Rotation2d.fromDegrees(70)); // Close to our arducam
        simCameraProperties.setCalibError(0.125, 0.04); // Unsure how accurate this is
        simCameraProperties.setFPS(40); // Should be close enough
        simCameraProperties.setAvgLatencyMs(40); // Should be close enough
        simCameraProperties.setLatencyStdDevMs(5); // Should be close enough
        cameraSim = new PhotonCameraSim(Camera, simCameraProperties);
        systemSim.addCamera(cameraSim, camTransform);

        // Enable the raw and processed streams. These are enabled by default.
        cameraSim.enableRawStream(true);
        cameraSim.enableProcessedStream(true);

        // Enable drawing a wireframe visualization of the field to the camera streams.
        // This is extremely resource-intensive and is disabled by default.
        cameraSim.enableDrawWireframe(true);
        cameraSim.setMaxSightRange(5);
    }

    public void updateSim(Pose2d currentRobotPose) {
        systemSim.update(currentRobotPose);
    }

    public void changePipeline(CameraPipeline pipeline) {
        currentPipeline = pipeline;
        switch (pipeline) {
            default:
            case AprilTag:
                Camera.setPipelineIndex(0);
                break;
            case Coral:
                // Camera.setPipelineIndex(0);
                break;

        }
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
        if (currentPipeline == CameraPipeline.AprilTag) {
            return getEstimatedGlobalPose(Camera, PoseEstimator);
        }
        return null;
    }
    public Pose2d Coral = new Pose2d();
    public Optional<Pose2d> getCoral() {
        if (currentPipeline == CameraPipeline.Coral) {
            try {

                Coral = Optional.ofNullable(
                    (new Pose3d(poseSupplier.get().getX(), poseSupplier.get().getY(), 0.0,
                            new Rotation3d(poseSupplier.get().getRotation()))
                            .transformBy(Camera.getAllUnreadResults().get(0).targets.get(0).getBestCameraToTarget()
                                    .plus(new Transform3d(Inches.of(-24), Inches.zero(), Inches.zero(),
                                            new Rotation3d()))))
                            .toPose2d()).orElse(Coral);
                return Optional.ofNullable(
                    (new Pose3d(poseSupplier.get().getX(), poseSupplier.get().getY(), 0.0,
                            new Rotation3d(poseSupplier.get().getRotation()))
                            .transformBy(Camera.getAllUnreadResults().get(0).targets.get(0).getBestCameraToTarget()
                                    .plus(new Transform3d(Inches.of(-24), Inches.zero(), Inches.zero(),
                                            new Rotation3d()))))
                            .toPose2d());

                // (new Pose3d(poseSupplier.get().getX(), poseSupplier.get().getY(), 0.0,
                // new Rotation3d(poseSupplier.get().getRotation()))
                // .transformBy(Camera.getAllUnreadResults().get(0).targets.get(0).getBestCameraToTarget())
                // .plus(new Transform3d(Inches.of(-24), Inches.zero(), Inches.zero(), new
                // Rotation3d())))
                // .toPose2d());
            } catch (Exception e) {
                // e.printStackTrace();
            }

        } 
                return Optional.empty();
    }

    public CameraPipeline getCameraPipeline() {
        return currentPipeline;
    }

}
