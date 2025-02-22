package frc.robot.subsystems.Arm;

import frc.robot.Constants.ArmConstants;
import frc.robot.Utilities.Elastic;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Rotations;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.WidgetType;
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
    private Angle lastSetpoint = Rotations.of(0);

    public ArmSubsystem() {
        Commands = new ArmCommands();
        m_Encoder = new DutyCycleEncoder(ArmConstants.IDs.Encoder);
        wrist = new TalonFX(ArmConstants.IDs.Wrist);
        wristConfig = new TalonFXConfiguration();
        wristConfig.CurrentLimits.StatorCurrentLimit = 20/2;
        wristConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        mmVolt = new MotionMagicVoltage(0);
        mmConfig = new MotionMagicConfigs();
        mmConfig = wristConfig.MotionMagic;
        mmConfig.MotionMagicCruiseVelocity = 800;
        mmConfig.MotionMagicAcceleration = 1000;
        

        var slot0 = wristConfig.Slot0;
        slot0.kA = 0.0; 
        slot0.kG = 0.02; //0.017
        slot0.kV = 0.017; //0.029
        slot0.kP = 0.45 ; 
        slot0.kI = 0.0;
        slot0.kD = 0.0;    
        slot0.kS = 0.0;
        wristConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        wristConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        slot0.GravityType = GravityTypeValue.Arm_Cosine;
        wrist.getConfigurator().apply(wristConfig);
        

        m_Encoder.setInverted(true);

        // Set initial value, shouldn't need to change later.
        wrist.setPosition(getAngle());
        SmartDashboard.putData("Brake Arm", Commands.brakeMotor(true));
        SmartDashboard.putData("Coast Arm", Commands.brakeMotor(false));
    }
    /**
     * Returns the encoder position in output angle
     */
    public Angle getPosition() {
        return Rotations.of(m_Encoder.get()).minus(ArmConstants.Offset);
    }
    /**
     * Returns the encoder position in input angle(motor) rotations
     */
    public Angle getAngle() {
        return (getPosition().times(ArmConstants.GearRatio));
    }

    public boolean atSetpoint() {
        return wrist.getPosition().getValue().isNear(lastSetpoint, Rotations.of(2));
    }

    public void runAngle(Angle angle) {
        // angle is in output degrees
        if (angle.lt(Degrees.of(-90))){
            angle = Degrees.of(-90);
        } else if (angle.gt(Degrees.of(90))){
            angle = Degrees.of(90);
        }
        lastSetpoint = angle.times(ArmConstants.GearRatio);
        wrist.setControl(mmVolt.withPosition(lastSetpoint));
    }

    @Override
    public void periodic() {
        // wrist.setPosition(getPosition());

        // This should result in less stuttering when we set a new angle
        if (!wrist.getPosition().getValue().isNear(getAngle(), Rotations.of(2))) {
            wrist.setPosition(getAngle());
        }
        //108 -> 19:30 = 170.5:1
        SmartDashboard.putNumber("WristAngleMotor", wrist.getPosition().getValueAsDouble());
        SmartDashboard.putNumber("WristAngleEncoder", getAngle().in(Rotations));
        // Shuffleboard.getTab("Teleoperated").add("WristAngleOutput", getPosition().in(Degrees)).withWidget(BuiltInWidgets.kDial);
        SmartDashboard.putNumber("WristAngleOutput", getPosition().in(Degrees));

        SmartDashboard.putNumber("WristSetpoint", lastSetpoint.in(Rotations));
        SmartDashboard.putNumber("WristCurrentStator", wrist.getStatorCurrent().getValueAsDouble());
        SmartDashboard.putNumber("WristCurrentStall", wrist.getMotorStallCurrent().getValueAsDouble());
        SmartDashboard.putNumber("WristCurrentSupply", wrist.getSupplyCurrent().getValueAsDouble());
        SmartDashboard.putNumber("WristCurrentTorque", wrist.getTorqueCurrent().getValueAsDouble());
        
    }



    public class ArmCommands {
        public Command L1() {
            return runOnce(() -> runAngle(ArmConstants.Setpoints.L1));
        }

        public Command L2() {
            return runOnce(() -> runAngle(ArmConstants.Setpoints.L2));
        }

        public Command L3() {
            return L2();
        }

        public Command L4() {
            return runOnce(() -> runAngle(ArmConstants.Setpoints.L4));
        }

        public Command Intake() {
            return runOnce(() -> runAngle(ArmConstants.Setpoints.Intake));
        }

        public Command GroundIntake() {
            return runOnce(() -> runAngle(ArmConstants.Setpoints.GroundIntake));
        }
        public Command changeSetpointBy(Angle D) {
            return runOnce(() -> {
                runAngle(lastSetpoint.plus(D));
            });
        }
        public Command brakeMotor(boolean brake){
            return runOnce(() -> {
                if (brake){
                    wristConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
                    wrist.getConfigurator().apply(wristConfig);
                } else {
                    wristConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
                    wrist.getConfigurator().apply(wristConfig);
                }
                
            }).ignoringDisable(true);
        }
    }
}
