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
import com.ctre.phoenix6.signals.NeutralModeValue;

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
        mmConfig.MotionMagicCruiseVelocity = 800;
        mmConfig.MotionMagicAcceleration = 1000;

        var slot0 = wristConfig.Slot0;
        slot0.kA = 0.0; // TODO find values
        slot0.kG = 0.01953125;
        slot0.kV = 0.04;
        slot0.kP = .1; // TODO tune values
        slot0.kI = 0.0;
        slot0.kD = 0.01;    
        slot0.kS = 0.0;
        wristConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        slot0.GravityType = GravityTypeValue.Arm_Cosine;
        wrist.getConfigurator().apply(wristConfig);

        m_Encoder.setInverted(true);

        // Set initial value, shouldn't need to change later.
        wrist.setPosition(getAngle());
    }

    public double getPosition() {
        return m_Encoder.get();
    }

    public Angle getAngle() {
        return (Rotations.of(getPosition()).minus(ArmConstants.Offset)).times(180);
    }

    public boolean atSetpoint() {
        return wrist.getPosition().getValue().isNear(lastSetpoint, Degrees.of(1));
    }

    public void runAngle(Angle angle) {
        // angle is in output degrees
        lastSetpoint = angle.times(180);
        wrist.setControl(mmVolt.withPosition(lastSetpoint));
    }

    @Override
    public void periodic() {
        // wrist.setPosition(getPosition());

        // This should result in less stuttering when we set a new angle
        if (!wrist.getPosition().getValue().isNear(getAngle(), Rotations.of(1.5))) {
            wrist.setPosition(getAngle());
        }
        //108 -> 19:30 = 170.5:1
        SmartDashboard.putNumber("WristAngleMotor", wrist.getPosition().getValueAsDouble());
        SmartDashboard.putNumber("WristAngleEncoder", getAngle().in(Rotations));
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
