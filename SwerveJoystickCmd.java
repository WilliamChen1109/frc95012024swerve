package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.SwerveSubsystem;

public class SwerveJoystickCmd extends Command{
    private final SwerveSubsystem swerveSubsystem;
    private final Supplier<Double> xSpdFunction, ySpdFunction, turningSpdFunction;
    private final Supplier<Boolean> fieldOrientedFunction;
    private final SlewRateLimiter xLimiter, yLimiter, turningLimiter;

    private double xSpeed, ySpeed, turningSpeed;

    public SwerveJoystickCmd(SwerveSubsystem swerveSubsystem, Supplier<Double> xSpdFunction,
            Supplier<Double> ySpdFunction, Supplier<Double> turningSpdFunction, Supplier<Boolean> fieldOrientedFunction){
        this.swerveSubsystem = swerveSubsystem;
        this.xSpdFunction = xSpdFunction;
        this.ySpdFunction = ySpdFunction;
        this.turningSpdFunction = turningSpdFunction;
        this.fieldOrientedFunction = fieldOrientedFunction;
        this.xLimiter = new SlewRateLimiter(DriveConstants.kTeleDriveMaxAccelerationUnitsPerSecond);
        this.yLimiter = new SlewRateLimiter(DriveConstants.kTeleDriveMaxAccelerationUnitsPerSecond);
        this.turningLimiter = new SlewRateLimiter(DriveConstants.kTeleDriveMaxAngularAccelerationUnitsPerSecond);

        addRequirements(swerveSubsystem);
    }

    @Override 
    public void initialize(){
        System.out.println("SwerveJoystickCmd Started");
    }

    @Override
    public void execute(){
        // 接Supplier數值
        this.xSpeed = this.xSpdFunction.get();
        this.ySpeed = this.ySpdFunction.get();
        this.turningSpeed = this.turningSpdFunction.get();

        // 設定Deadband(換羅技飛行搖桿的話要重新再設定一個)
        xSpeed = Math.abs(xSpeed) > Constants.OIConstants.kDeadband ? xSpeed : 0;
        ySpeed = Math.abs(ySpeed) > Constants.OIConstants.kDeadband ? ySpeed : 0;
        turningSpeed = Math.abs(turningSpeed) > Constants.OIConstants.kDeadband ? turningSpeed : 0;

        // 平滑化
        xSpeed = xLimiter.calculate(xSpeed) * DriveConstants.kTeleDriveMaxSpeedMetersPerSecond;
        ySpeed = yLimiter.calculate(ySpeed) * DriveConstants.kTeleDriveMaxSpeedMetersPerSecond;
        turningSpeed = turningLimiter.calculate(turningSpeed) * DriveConstants.kTeleDriveMaxAngularSpeedRadiansPerSecond;

        // 設定速度
        ChassisSpeeds chassisSpeeds;
        if(fieldOrientedFunction.get()){
            chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, turningSpeed, swerveSubsystem.getRotation2d());
        }
        else{
            chassisSpeeds = new ChassisSpeeds(xSpeed, ySpeed, turningSpeed);
        }

        SwerveModuleState[] moduleStates = DriveConstants.kSwerveKinematics.toSwerveModuleStates(chassisSpeeds);

        swerveSubsystem.setModuleStates(moduleStates);
        
        printMoudleRadOffset();
    }

    @Override
    public void end(boolean interrupted){
        swerveSubsystem.stopModules();
        System.out.println("SwerveJoystickCmd finished");
    }

    @Override
    public boolean isFinished(){
        return false;
    }

    private void printMoudleRadOffset(){
        System.out.println("FrontLeft: " + swerveSubsystem.getModuleAbsoluteEncoderRad()[0]);
        System.out.println("FrontRight: " + swerveSubsystem.getModuleAbsoluteEncoderRad()[1]);
        System.out.println("BackLeft: " + swerveSubsystem.getModuleAbsoluteEncoderRad()[2]);
        System.out.println("BackRight: " + swerveSubsystem.getModuleAbsoluteEncoderRad()[3]);
    }
    // 1. 用手把輪子搬正
    // 2. 讀Offset的值，直接填到Conatants.java裡面(注意回傳的順序)
}
