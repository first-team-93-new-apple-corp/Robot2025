package frc.robot.subsystems.VisionIO;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class VisionSubsystem extends SubsystemBase {
    NetworkTable m_limelight = NetworkTableInstance.getDefault().getTable("limelight");

    double tx, ty, ta, tid, ts, tl, cl, tv;
    double[] targetpose_robotspace, botpose, botpose_wpiblue;

    Pose2d pose;

    public VisionSubsystem() {
        tx = m_limelight.getEntry("tx").getDouble(0);
        ty = m_limelight.getEntry("ty").getDouble(0);
        tv = m_limelight.getEntry("tv").getDouble(0);
        ta = m_limelight.getEntry("ta").getDouble(0);
        tid = m_limelight.getEntry("tid").getDouble(0);
        ts = m_limelight.getEntry("ts").getDouble(0);
        tl = m_limelight.getEntry("tl").getDouble(0);
        cl = m_limelight.getEntry("cl").getDouble(0);
        targetpose_robotspace = m_limelight.getEntry("targetpose_robotspace").getDoubleArray(new double[6]);
        botpose_wpiblue = m_limelight.getEntry("botpose_wpiblue").getDoubleArray(new double[6]);
        pose = new Pose2d();
    }

    public boolean hasTargets() {
        updateValues();
        if (tv == 1) {
            return true;
        } else
            return false;
    }

    public void LimeLightOn() {
        m_limelight.getEntry("ledMode").setNumber(3);
    }

    public void LimeLightOff() {
        m_limelight.getEntry("ledMode").setNumber(0);
    }

    public void updateValues() {
        tx = m_limelight.getEntry("tx").getDouble(0);
        tv = m_limelight.getEntry("tv").getDouble(0);
        ty = m_limelight.getEntry("ty").getDouble(0);
        tid = m_limelight.getEntry("tid").getDouble(0);
        ts = m_limelight.getEntry("ts").getDouble(0);
        tl = m_limelight.getEntry("tl").getDouble(0);
        cl = m_limelight.getEntry("cl").getDouble(0);
        targetpose_robotspace = m_limelight.getEntry("targetpose_robotspace").getDoubleArray(new double[6]);
        botpose_wpiblue = m_limelight.getEntry("botpose_wpiblue").getDoubleArray(new double[6]);
    }   

    public Pose2d getPose() {
        updateValues();
        pose = new Pose2d(botpose_wpiblue[0], botpose_wpiblue[1], new Rotation2d(Math.toRadians(botpose_wpiblue[5])));
        return pose;
    }

    @Override
    public void periodic() {
        updateValues();
        SmartDashboard.putBoolean("Has targets", hasTargets());
    }
}