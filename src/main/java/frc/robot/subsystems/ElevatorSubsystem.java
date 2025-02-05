package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Inches;

import java.lang.reflect.Method;

import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConstants;

public class ElevatorSubsystem extends SubsystemBase{
    // Kraken x60 (Controlled by TalonFX) 12-1 gear
    // 18-toothed sprocket 1-1 chain
    // 32.9 in max extension
    // 
    public TalonFX elevatorMotor = new TalonFX(ElevatorConstants.elevatorMotorID);
    MotionMagicVoltage motionMagic = new MotionMagicVoltage(0);
    public ElevatorCommands elevatorCommands = new ElevatorCommands();
    public static final ElevatorSim m_elevatorSim =
    new ElevatorSim(DCMotor.getKrakenX60(1),
    ElevatorConstants.kElevatorGearing,
    ElevatorConstants.kCarriageMass,
    ElevatorConstants.kElevatorDrumRadius,
    ElevatorConstants.kMinElevatorHeightMeters,
    ElevatorConstants.kMaxElevatorHeightMeters,
    true,
    0,
    0.01,
    0.0);

    public double convertToTicks(Distance inches) {
        return inches.magnitude() * ElevatorConstants.revolutionsPerInch*ElevatorConstants.ticksPerRevolution;
    }
    public void setSpeed(double speed) {
        elevatorMotor.set(speed);
    }
    
    public class ElevatorCommands {
        public Command l1() {
            
            // return runOnce(() -> elevatorMotor.setControl(motionMagic.withPosition(convertToTicks(ElevatorConstants.l1Setpoint))));
            return runOnce(() -> setSpeed(1));

    
        }
        public Command l2() {
            return runOnce(() -> elevatorMotor.setControl(motionMagic.withPosition(convertToTicks(ElevatorConstants.l2Setpoint))));
    
        }
        public Command l3() {
            return runOnce(() -> elevatorMotor.setControl(motionMagic.withPosition(convertToTicks(ElevatorConstants.l3Setpoint))));
    
        }
        public Command l4() {
            return runOnce(() -> elevatorMotor.setControl(motionMagic.withPosition(convertToTicks(ElevatorConstants.l4Setpoint))));
    
        }
    }
    }

   
