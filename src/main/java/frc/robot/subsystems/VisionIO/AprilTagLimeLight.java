package frc.robot.subsystems.VisionIO;

import java.util.Optional;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

public class AprilTagLimeLight{
    private final NetworkTable Limelight;
    public AprilTagLimeLight(String name){
        Limelight = NetworkTableInstance.getDefault().getTable(name);
    }
    public Optional<Pose2d> getCurrentPoseEstimation() {
        return null;
    }

    public boolean hasTargets() {
        return Limelight.getEntry("tv").getDouble(0) > 0;
    }
    
}
