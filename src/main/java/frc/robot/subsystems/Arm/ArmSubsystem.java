package frc.robot.subsystems.Arm;

import frc.robot.Constants.ArmConstants;
import frc.robot.commands.intake;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Rotations;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
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
    private Angle lastSetpoint;

    public ArmSubsystem() {
        Commands = new ArmCommands();
        m_Encoder = new DutyCycleEncoder(ArmConstants.IDs.Encoder);
        wrist = new TalonFX(ArmConstants.IDs.Wrist);
        wristConfig = new TalonFXConfiguration();
        mmVolt = new MotionMagicVoltage(0);
        mmConfig = new MotionMagicConfigs();
        mmConfig = wristConfig.MotionMagic;
        mmConfig.MotionMagicCruiseVelocity = 80;
        mmConfig.MotionMagicAcceleration = 160;

        var slot0 = wristConfig.Slot0;
        slot0.kA = 0.0; // TODO find values
        slot0.kG = -0.07;
        slot0.kV = 0.1;
        slot0.kP = 0.3; // TODO tune values
        slot0.kI = 0.0;
        slot0.kD = 0.0;
        slot0.kS = 0.0;
        slot0.GravityType = GravityTypeValue.Arm_Cosine;
        wrist.getConfigurator().apply(wristConfig);

        m_Encoder.setInverted(true);

        // Set initial value, shouldn't need to change later.
        wrist.setPosition(getAngle());
    }

    public double getPosition() {
        return m_Encoder.get() * 180;
    }

    public Angle getAngle() {
        return Degrees.of(getPosition()).minus(ArmConstants.Offset);
    }

    public boolean atSetpoint() {
        return wrist.getPosition().getValue().isNear(lastSetpoint, Degrees.of(2));
    }

    public void runAngle(Angle angle) {
        lastSetpoint = angle;
        wrist.setControl(mmVolt.withPosition(angle));
    }

    @Override
    public void periodic() {
        // wrist.setPosition(getPosition());

        // This should result in less stuttering when we set a new angle
        if (!wrist.getPosition().getValue().isNear(getAngle(), Degrees.of(5))) {
            wrist.setPosition(getAngle());
        }
        //108 -> 19:30 = 170.5:1
        SmartDashboard.putNumber("WristAngleMotor", wrist.getPosition().getValueAsDouble());
        SmartDashboard.putNumber("WristAngleEncoder", getAngle().magnitude());
    }

    public class ArmCommands {
        public Command L1() {
            return run(() -> runAngle(ArmConstants.Setpoints.L1));
        }

        public Command L2() {
            return run(() -> runAngle(ArmConstants.Setpoints.L2));
        }

        public Command L3() {
            return L2();
        }

        public Command L4() {
            return run(() -> runAngle(ArmConstants.Setpoints.L4));
        }

        public Command Intake() {
            return run(() -> runAngle(ArmConstants.Setpoints.Intake));
        }
    }
}
