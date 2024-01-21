package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.ShooterCmd;
import frc.robot.commands.SwerveJoystickCmd;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.SwerveSubsystem;

public class RobotContainer {
  private final XboxController stick = new XboxController(Constants.OperatorConstants.kDriverControllerPort);
  
  private final SwerveSubsystem swerveSubsystem = new SwerveSubsystem();
  private final Shooter shooter = new Shooter();

  public RobotContainer() {
    swerveSubsystem.setDefaultCommand(new SwerveJoystickCmd(swerveSubsystem,
      () -> stick.getLeftY(),
      () -> stick.getLeftX(),
      () -> stick.getRightX(),
      () -> !stick.getAButton()));

    configureBindings();
  }

  private void configureBindings() {
    new JoystickButton(stick, Button.kX.value).onTrue(new InstantCommand(() -> swerveSubsystem.zeroHeading(),swerveSubsystem));
    new JoystickButton(stick, Button.kY.value).whileTrue(new ShooterCmd(shooter, 0.7));
    new JoystickButton(stick, Button.kB.value).whileTrue(new ShooterCmd(shooter, -0.5));
  }

  public Command getAutonomousCommand() {
    return null;
  }
}
