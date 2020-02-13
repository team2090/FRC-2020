/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;
import frc.robot.Robot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SwerveSubsystem;


public class TeleOpDriveCommand extends CommandBase {
  private final SwerveSubsystem robotDrive;
  public TeleOpDriveCommand(SwerveSubsystem swerve) {
    robotDrive = swerve;
    addRequirements(swerve);
  }

  // Called just before this Command runs the first time
  @Override
  public void initialize() {
    robotDrive.generateWheels();
    robotDrive.initWheels();
    SmartDashboard.putString("State", "Teleop Init");
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  public void execute() {
    SmartDashboard.putString("State", "Robot Running");
    double forward = deadband(Robot.controls.getForward());
    double strafe = deadband(Robot.controls.getStrafe());
    double yaw = deadband(Robot.controls.getYaw());

    if (Robot.controls.slowDriveMode()) {
      forward /= 2;
      strafe /= 2;
      yaw /= 2;
    }

    // Mode changing
    if (Robot.controls.robotOriented()) {
      SmartDashboard.putString("Mode", "Robot Oriented");
      robotDrive.setFieldOriented(false);
    } else if (Robot.controls.fieldOriented()) {
      SmartDashboard.putString("Mode", "Field Oriented");
      robotDrive.setFieldOriented(true);
    }

    if (Robot.controls.robotOrientedForward()) {
      robotDrive.setFieldOriented(false);
      robotDrive.drive(0.3, 0, 0);
      robotDrive.setFieldOriented(true);
    }

    SmartDashboard.putNumber("Forward", forward);
    SmartDashboard.putNumber("Strafe", strafe);
    SmartDashboard.putNumber("Yaw", yaw);

    robotDrive.drive(forward, strafe, yaw);
  }

  public double deadband(double value) {
    if (Math.abs(value) > 0.1) {
      return value;
    } else {
      return 0;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    //robotDrive.drive(0.0, 0.0, 0.0);
    robotDrive.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
