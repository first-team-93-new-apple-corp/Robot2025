package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import java.lang.reflect.Method;

import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.AngleUnit;
import edu.wpi.first.units.AngularVelocityUnit;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConstants;

public class ElevatorSubsystem extends SubsystemBase{
    // Kraken x60 (Controlled by TalonFX) 12-1 gear
    // 18-toothed sprocket 1-1 chain
    // 32.9 in max extension
    // 
    public TalonFX elevatorMotor;
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

    public ElevatorSubsystem() {
        elevatorMotor = new TalonFX(ElevatorConstants.elevatorMotorID);
    }
    public AngularVelocityUnit convertToAngle(Distance inches) { // Math needs work
        return RotationsPerSecond.of(inches.magnitude() * ElevatorConstants.revolutionsPerInch*ElevatorConstants.ticksPerRevolution).baseUnit();
    }
    public void setSpeed(double speed) {
        elevatorMotor.set(speed);
    }
    @Override
    public void simulationPeriodic() {
        System.out.println(motionMagic.Position);
        // m_elevatorSim.setInput(elevatorMotor.getMotorVoltage().getValueAsDouble() * RobotController.getBatteryVoltage());
        // m_elevatorSim.update(0.020);
    
        // RoboRioSim.setVInVoltage(
        //     BatterySim.calculateDefaultBatteryLoadedVoltage(m_elevatorSim.getCurrentDrawAmps()));
    
    }
    public class ElevatorCommands {
        public Command l1() {
            
            return runOnce(() -> elevatorMotor.setControl(motionMagic.withPosition(30)));
            // return runOnce(() -> setSpeed(1));

        
        }
        // public Command l2() {
            
        //     return runOnce(() -> elevatorMotor.setControl(motionMagic.withPosition(convertToAngle(ElevatorConstants.l2Setpoint))));
    
        // }
        // public Command l3() {
        //     return runOnce(() -> elevatorMotor.setControl(motionMagic.withPosition(convertToAngle(ElevatorConstants.l3Setpoint))));
    
        // }
        // public Command l4() {
        //     return runOnce(() -> elevatorMotor.setControl(motionMagic.withPosition(convertToAngle(ElevatorConstants.l4Setpoint))));
    
        // }
        public Command stopElevator() {
            return runOnce(() -> setSpeed(0));
    
        }
    }
}

   
