package frc.robot.subsystems.Arm;

import frc.robot.Constants.CTRE;
import frc.robot.Constants.Inputs;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ArmSubsystem extends SubsystemBase {
    private TalonFX wrist;
    private TalonFXConfiguration wristConfig;

    private MotionMagicVoltage mmVolt;
    private MotionMagicConfigs mmConfig;

    private DutyCycleEncoder m_Encoder;
    public ArmCommands Commands;
    // Setpoints
    private double restIntake, Low, Mid, High;
    // Limts
    private double lowLimit, highLimit;

    public ArmSubsystem() {
        Commands = new ArmCommands();
        m_Encoder = new DutyCycleEncoder(Inputs.DIO.ThroughBoreEncoder);
        wrist = new TalonFX(CTRE.Wrist);
        wristConfig = new TalonFXConfiguration();
        mmVolt = new MotionMagicVoltage(0);
        mmConfig = new MotionMagicConfigs();
        mmConfig = wristConfig.MotionMagic;
        mmConfig.MotionMagicCruiseVelocity = 80;
        mmConfig.MotionMagicAcceleration = 160;

        var slot0 = wristConfig.Slot0;
        slot0.kA = 0.0; // TODO find values
        slot0.kG = 0.0;
        slot0.kV = 0.0;
        slot0.kP = 1.0; // TODO tune values
        slot0.kI = 0.0;
        slot0.kD = 0.0;
        slot0.kS = 0.0;
        wrist.getConfigurator().apply(wristConfig);
        lowLimit = 0.0;
        highLimit = 0.0;

        High = 10.0; //
        Mid = 0.0; //
        Low = 0.0; //
        restIntake = 0.0; // -90 degree from ground
    }

    public double getPosition() {
        return m_Encoder.get();
    }

    public void runAngle(double angle) {
        wrist.setControl(mmVolt.withPosition(angle));
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("WristAngle", m_Encoder.getFrequency());
    }

    public class ArmCommands {
        public Command runHigh() {
            return run(() -> runAngle(High));
        }

        public Command runMid() {
            return run(() -> runAngle(Mid));
        }

        public Command runLow() {
            return run(() -> runAngle(Low));
        }

        public Command runRestIntake() {
            return run(() -> runAngle(restIntake));
        }

    }
}
