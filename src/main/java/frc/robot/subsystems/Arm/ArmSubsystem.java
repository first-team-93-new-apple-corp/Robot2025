package frc.robot.subsystems.Arm;

import frc.robot.Constants.CTRE;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ArmSubsystem extends SubsystemBase{
    private TalonFX wrist;
    private TalonFXConfiguration wristConfig;

    private MotionMagicConfigs wristPID;

    private double setpointHigh, setpointMedium, setpointLow, setpointIntake, setpointStow;

    public ArmSubsystem(){
        wrist = new TalonFX(CTRE.Wrist);    
        wristConfig = new TalonFXConfiguration();
        wristConfig.Slot0.withKP(0);

        SmartDashboard.putString("WristStage", "Null");
    }
    public void runAngle(double angle){
        double runAngle = 0;
        wrist.set(MathUtil.clamp(runAngle,0,0));
    }

    @Override
    public void periodic(){
        SmartDashboard.putNumber("WristAngle", wrist.get());    
    }
    public class Commands{
        public Command runPoseHigh(){
            SmartDashboard.putString("WristStage", "High");
            return run(() -> runAngle(setpointHigh));
        }
        public Command runPoseMedium(){
            SmartDashboard.putString("WristStage", "Medium");
            return run(() -> runAngle(setpointMedium));
        }
        public Command runPoseLow(){
            SmartDashboard.putString("WristStage", "Low");
            return run(() -> runAngle(setpointLow));
        }
        public Command runPoseStow(){
            SmartDashboard.putString("WristStage", "Stow");
            return run(() -> runAngle(setpointStow));
        }
        public Command runPoseIntake(){
            SmartDashboard.putString("WristStage", "Intake");
            return run(() -> runAngle(setpointIntake));
        }
    }
}
