package frc.robot.subsystems.Grabber;

import frc.robot.Constants.CTRE;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;

public class GrabberSubsystem implements Subsystem{
    SparkMax motor;

    private double intakeSpeed;
    private double outtakeSpeed;
    public GrabberCommands Commands = new GrabberCommands();
    public GrabberSubsystem(){
        motor = new SparkMax(CTRE.Grabber, MotorType.kBrushless);
        //GrabberSubsystem Constructor
        outtakeSpeed  = -0.2;
        intakeSpeed = 0.5;
    }
    @Override
    public void periodic(){
        //GrabberSubsystem periodic
    }
    public void setSpeed(double speed){
        motor.set(speed);
        if(speed == 0) motor.stopMotor();
    }
    public class GrabberCommands{
        public Command intake(){
            return runOnce(() -> setSpeed(intakeSpeed));
        }
        public Command outtake() {
            return runOnce(() -> setSpeed(outtakeSpeed));
        }
        public Command brake(){
            return runOnce(() -> setSpeed(0));
        }
    }

}
