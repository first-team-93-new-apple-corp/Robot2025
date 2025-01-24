package frc.robot.subsystems.VisionIO;



import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.Inputs.Cameras;

public class Vision extends SubsystemBase {
    private PhotonCamera frontCam;
    private PhotonCamera rearCam;
    private PhotonPipelineResult frontResult;
    private PhotonPipelineResult rearResult;
    private PhotonTrackedTarget frontTarget;
    private PhotonTrackedTarget rearTarget;
    

    public Vision() {
        frontCam = new PhotonCamera(Cameras.frontCam.CamName);
    }

    @Override
    public void periodic() {
        
    }
}
