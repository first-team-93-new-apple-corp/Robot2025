package frc.robot.subsystems.VisionIO;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.pathplanner.lib.commands.PathfindingCommand;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Vision extends SubsystemBase {
    private NetworkTableEntry limelight = NetworkTableInstance.getDefault().getTable("limelight-front")
            .getEntry("botpose_targetspace");
    // Pigeon2 pigeon2;
    private Double[] values;
    private Double[] defaultValues;
    private double rotation;
    PhotonCamera camera = new PhotonCamera("Microsoft_LifeCam_HD-3000");
    PhotonPipelineResult result;
    PhotonTrackedTarget target;
    double pitch;
    double yaw = 0.0;
    double area;
    private PIDController pid = new PIDController(0.05, 0, 0);
    private PIDController rotate = new PIDController(0.05, 0, 0);

    public Vision() {

        // this.pigeon2 = pigeon2;
        defaultValues = new Double[6];
        defaultValues[0] = 0.;
        defaultValues[1] = 0.;
        defaultValues[2] = 0.;
        defaultValues[3] = 0.;
        defaultValues[4] = 0.;
        defaultValues[5] = 0.;
        rotate.setSetpoint(0);
        rotate.setTolerance(2, 0.2);
    }

    public double pointToCalc() {
        return -pid.calculate(rotation, 0);
    }

    public double turnToNote() {
        return -rotate.calculate(yaw);
    }

    @Override
    public void periodic() {
        result = camera.getLatestResult();
        if (result.hasTargets()) {
            target = result.getBestTarget();
            
            yaw = target.getYaw();
        }
        values = limelight.getDoubleArray(defaultValues);
        rotation = values[4];
        SmartDashboard.putNumberArray("limelight values", values);
        SmartDashboard.putNumber("rotation", rotation);
        SmartDashboard.putNumber("yaw", yaw);
    }
}
