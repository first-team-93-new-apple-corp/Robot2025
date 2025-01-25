package frc.robot.subsystems.VisionIO;

import java.util.ArrayList;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Inputs.Cameras;

public class Vision extends SubsystemBase {
    private PhotonCamera frontCam;
    private PhotonCamera rearCam;
    private ArrayList<PhotonPipelineResult> frontResult;
    private ArrayList<PhotonPipelineResult> rearResult;
    private PhotonTrackedTarget frontTarget;
    private PhotonTrackedTarget rearTarget;
    private PhotonPoseEstimator frontPoseEstimator;
    private PhotonPoseEstimator rearPoseEstimator;

    public Vision() {
        frontCam = new PhotonCamera(Cameras.frontCam.CamName);
        rearCam = new PhotonCamera(Cameras.rearCam.CamName);
        frontPoseEstimator = new PhotonPoseEstimator(AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape),
                PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, Cameras.frontCam.CamPose);
        rearPoseEstimator = new PhotonPoseEstimator(AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape),
                PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, Cameras.rearCam.CamPose);

    }

    public Optional<EstimatedRobotPose> getEstimatedGlobalPose(Pose2d prevEstimatedRobotPose,
            PhotonPoseEstimator photonPoseEstimator, PhotonPipelineResult target) {
        photonPoseEstimator.setReferencePose(prevEstimatedRobotPose);
        return photonPoseEstimator.update(target);
    }

    public void getResults() {
        frontResult = new ArrayList<>(frontCam.getAllUnreadResults());
        rearResult = new ArrayList<>(rearCam.getAllUnreadResults());

    }

    public boolean camHasTarget(ArrayList<PhotonPipelineResult> result) {
        if (result.size() > 0) {
            return true;
        } else
            return false;
    }

    public boolean hasTargets() {
        if (camHasTarget(frontResult) || camHasTarget(rearResult)) {
            return true;
        } else
            return false;
    }

    public void smartDash() {
        // Put all smartdash stuff in here
        SmartDashboard.putBoolean("Has Targets?", hasTargets());

    }

    @Override
    public void periodic() {
        smartDash();

    }
}
