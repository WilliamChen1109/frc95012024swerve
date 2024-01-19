package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class SwerveSubsystem extends SubsystemBase{
    private final SwerveModule frontLeft = new SwerveModule(
            Constants.DriveConstants.kFrontLeftDriveMotorPort,
            Constants.DriveConstants.kFrontLeftTurningMotorPort, 
            Constants.DriveConstants.kFrontLeftDriveEncoderReversed, 
            Constants.DriveConstants.kFrontLeftTurningEncoderReversed, 
            Constants.DriveConstants.kFrontLeftDriveAbsoluteEncoderPort, 
            Constants.DriveConstants.kFrontLeftAbsoluteEncoderOffsetRad, 
            Constants.DriveConstants.kFrontLeftAbsoluteEncoderReversed);

    private final SwerveModule frontRight = new SwerveModule(
            Constants.DriveConstants.kFrontRightDriveMotorPort,
            Constants.DriveConstants.kFrontRightTurningMotorPort, 
            Constants.DriveConstants.kFrontRightDriveEncoderReversed, 
            Constants.DriveConstants.kFrontRightTurningEncoderReversed, 
            Constants.DriveConstants.kFrontRightDriveAbsoluteEncoderPort, 
            Constants.DriveConstants.kFrontRightAbsoluteEncoderOffsetRad, 
            Constants.DriveConstants.kFrontRightAbsoluteEncoderReversed);

    private final SwerveModule backLeft = new SwerveModule(
            Constants.DriveConstants.kBackLeftDriveMotorPort,
            Constants.DriveConstants.kBackLeftTurningMotorPort, 
            Constants.DriveConstants.kBackLeftDriveEncoderReversed, 
            Constants.DriveConstants.kBackLeftTurningEncoderReversed, 
            Constants.DriveConstants.kBackLeftDriveAbsoluteEncoderPort, 
            Constants.DriveConstants.kBackLeftAbsoluteEncoderOffsetRad, 
            Constants.DriveConstants.kBackLeftAbsoluteEncoderReversed);

    private final SwerveModule backRight = new SwerveModule(
            Constants.DriveConstants.kBackRightDriveMotorPort,
            Constants.DriveConstants.kBackRightTurningMotorPort, 
            Constants.DriveConstants.kBackRightDriveEncoderReversed, 
            Constants.DriveConstants.kBackRightTurningEncoderReversed, 
            Constants.DriveConstants.kBackRightDriveAbsoluteEncoderPort, 
            Constants.DriveConstants.kBackRightAbsoluteEncoderOffsetRad, 
            Constants.DriveConstants.kBackRightAbsoluteEncoderReversed);

    private final AHRS gyro = new AHRS(SPI.Port.kMXP);

    private final SwerveDriveOdometry odometer = new SwerveDriveOdometry(Constants.DriveConstants.kSwerveKinematics, new Rotation2d(gyro.getAngle()), getModulePositions());
    
    // Constructure
    public SwerveSubsystem(){
        new Thread(() -> {
            try {
                Thread.sleep(1000);
                zeroHeading();
            } catch (Exception e){

            }
        }).start();
    }

    // 重制陀螺儀
    public void zeroHeading(){
        gyro.reset();
    }

    // 拿陀螺儀的值(鎖在360度內)
    public double getHeading(){
        return Math.IEEEremainder(gyro.getAngle(), 360);
    }

    // 拿取Rotation2D(從陀螺儀來)
    public Rotation2d getRotation2d(){
        return Rotation2d.fromDegrees(getHeading());
    }

    // 拿取Pose2d
    public Pose2d getPose(){
        return odometer.getPoseMeters();
    }

    //
    public SwerveModulePosition[] getModulePositions(){
        return new SwerveModulePosition[]{
            frontLeft.getPosition(),
            frontRight.getPosition(),
            backLeft.getPosition(),
            backRight.getPosition()
        };
    }

    //
    public void resetOdometry(Pose2d pose){
        odometer.resetPosition(getRotation2d(), null, pose);
    }

    public void stopModules(){
        frontLeft.stop();
        frontRight.stop();
        backLeft.stop();
        backRight.stop();
    }

    public void setModuleStates(SwerveModuleState[] desiStates){
        SwerveDriveKinematics.desaturateWheelSpeeds(desiStates, 1);
        frontLeft.setDesiredState(desiStates[0]);
        frontRight.setDesiredState(desiStates[1]);
        backLeft.setDesiredState(desiStates[2]);
        backRight.setDesiredState(desiStates[3]);
    }

    @Override
    public void periodic(){
        odometer.update(getRotation2d(), getModulePositions());
    }

    public void setPose(Pose2d pose){
        odometer.resetPosition(getRotation2d(), getModulePositions(), pose);
    }
}
