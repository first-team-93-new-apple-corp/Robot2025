package frc.robot.subsystems.VisionIO;

import java.util.ArrayList;
import java.util.Optional;

import org.jcp.xml.dsig.internal.dom.Utils;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Swerve.SwerveDriveSubsystem;
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
    private SwerveDriveSubsystem m_DriveSubsystem;

    public Vision(SwerveDriveSubsystem m_DriveSubsystem) {
        this.m_DriveSubsystem = m_DriveSubsystem;
        frontCam = new PhotonCamera(Cameras.frontCam.CamName);
        rearCam = new PhotonCamera(Cameras.rearCam.CamName);
        frontPoseEstimator = new PhotonPoseEstimator(AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape),
                PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, Cameras.frontCam.CamPose);
        rearPoseEstimator = new PhotonPoseEstimator(AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape),
                PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, Cameras.rearCam.CamPose);

    }

    // public Optional<EstimatedRobotPose> getEstimatedGlobalPose(Pose2d prevEstimatedRobotPose,
    //         PhotonPoseEstimator photonPoseEstimator, PhotonPipelineResult target) {
    //     photonPoseEstimator.setReferencePose(prevEstimatedRobotPose);
    //     return photonPoseEstimator.update(target);
    // }
    public Optional<EstimatedRobotPose> getEstimatedGlobalPoseFront(Pose2d prevEstimatedRobotPose) {
        frontPoseEstimator.setReferencePose(prevEstimatedRobotPose);
        return frontPoseEstimator.update();
    }
    public Optional<EstimatedRobotPose> getEstimatedGlobalPoseRear(Pose2d prevEstimatedRobotPose) {
        rearPoseEstimator.setReferencePose(prevEstimatedRobotPose);
        return rearPoseEstimator.update();
    }
    public void getResults() {
        frontResult = new ArrayList<>(frontCam.getAllUnreadResults());
        rearResult = new ArrayList<>(rearCam.getAllUnreadResults());
        double timeStamp = com.ctre.phoenix6.Utils.getCurrentTimeSeconds();
        updatePosition(
            new Pose2d(getEstimatedGlobalPoseFront(new Pose2d(0,0,null)).get().estimatedPose.getX(), 
            getEstimatedGlobalPoseFront(new Pose2d(0,0,null)).get().estimatedPose.getY(),
            getEstimatedGlobalPoseFront(new Pose2d(0,0,null)).get().estimatedPose.getRotation().toRotation2d())
            ,timeStamp,null);
    }
    
    public void updatePosition(Pose2d pose, double timestamp, Matrix<N3, N1> visionMeasurementStdDevs) {
        m_DriveSubsystem.addVisionMeasurement(pose, timestamp, visionMeasurementStdDevs);
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
