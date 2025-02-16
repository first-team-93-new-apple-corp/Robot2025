package frc.robot.subsystems.VisionIO;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.Constants.Inputs.Cameras.Camera;

public class CameraFactory {
    public Vision build(Supplier<Pose2d> poseSup, Camera Cam ){
        return new Vision(poseSup, Cam.CamName(), Cam.camTransform());
    }
} 