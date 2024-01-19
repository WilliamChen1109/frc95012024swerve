package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;

public class ShooterCmd extends Command{
    private Shooter shooter;
    private double speed;

    public ShooterCmd(Shooter shooter, double speed){
        this.shooter = shooter;
        this.speed = speed;

        addRequirements(shooter);
    }

    //初始化(進到Command會跑一次)
    @Override
    public void initialize(){
        System.out.println("ShooterCmd Started!");
    }

    //循環(進到Command後循環)
    @Override
    public void execute(){
        shooter.set(this.speed);
    }

    @Override
    public void end(boolean interrupted){
        shooter.stopMotor();
        System.out.println("ShooterCmd finished!");
    }

    @Override
    public boolean isFinished(){
        return false;
    }
}
