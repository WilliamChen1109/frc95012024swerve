package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase{
    private CANSparkMax IntakeMotor = new CANSparkMax(Constants.IntakeConstants.kIntakeMotorPort, MotorType.kBrushless);
    private CANSparkMax IntakeCtrlMotor = new CANSparkMax(Constants.IntakeConstants.kIntakeCtrlMotorPort, MotorType.kBrushless);

    public Intake(){
        IntakeMotor.restoreFactoryDefaults();
        IntakeCtrlMotor.restoreFactoryDefaults();
    }

    public void stopMotor(){
        IntakeMotor.stopMotor();
        IntakeCtrlMotor.stopMotor();
    }

    public void setIntakeMotor(double speed){
        IntakeMotor.set(speed);
    }

    public void setIntakeCtrlMotor(double speed){
        IntakeCtrlMotor.set(speed);
    }

    @Override
    public void periodic(){
    }
}
