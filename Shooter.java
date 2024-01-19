package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase{
    private CANSparkMax shooter = new CANSparkMax(0, MotorType.kBrushless);
    private double speed;

    public Shooter(){
        shooter.restoreFactoryDefaults();
    }

    public void stopMotor(){
        shooter.stopMotor();
    }

    public void set(double speed){
        this.speed = speed;
        shooter.set(speed);
    }

    @Override
    public void periodic(){
        System.out.println("shooterSpd:" + this.speed);
    }
}
