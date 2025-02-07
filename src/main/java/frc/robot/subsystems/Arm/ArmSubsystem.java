package frc.robot.subsystems.Arm;

import frc.robot.Constants.CTRE;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ArmSubsystem extends SubsystemBase {
    private TalonFX wrist;
    private TalonFXConfiguration wristConfig;
    public SingleJointedArmSim wristSim;
    private MotionMagicConfigs wristMM;
    public Commands Commands = new Commands();

    private double setpointHigh, setpointMedium, setpointLow, setpointIntake, setpointStow, lowLimit, highLimit;

    public ArmSubsystem(){
        wrist = new TalonFX(CTRE.Wrist);    
        wristConfig = new TalonFXConfiguration();
        wristSim = new SingleJointedArmSim(new DCMotor(setpointLow, setpointIntake, setpointHigh, lowLimit, highLimit, 0))
        var slot0 = wristConfig.Slot0;
        slot0.kA = 0.0; //TODO find values
        slot0.kG = 0.0;
        slot0.kV = 0.0;
        slot0.kP = 1.0; //TODO tune values
        slot0.kI = 0.0;
        slot0.kD = 0.0;
        slot0.kS = 0.0;
       
        lowLimit = 0.0;
        highLimit = 0.0;

        setpointHigh = 10.0;
        setpointMedium = 0.0;
        setpointLow = 0.0;
        setpointIntake = 0.0;
        setpointStow = 0.0;
    }

    public void defineSim() {

    }

    public void updateSim() {
    }

    public void runAngle(double angle) {
        // double runAngle = 0;
        // wrist.set(MathUtil.clamp(runAngle, lowLimit, highLimit));
        wrist.set(angle);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("WristAngle", wrist.getPosition().getValueAsDouble());
    }

    public class Commands {
        public Command runPoseHigh() {
            return run(() -> runAngle(setpointHigh));
        }

        public Command runPoseMedium() {
            return run(() -> runAngle(setpointMedium));
        }

        public Command runPoseLow() {
            return run(() -> runAngle(setpointLow));
        }

        public Command runPoseStow() {
            return run(() -> runAngle(setpointStow));
        }

        public Command runPoseIntake() {
            return run(() -> runAngle(setpointIntake));
        }
    }
}
