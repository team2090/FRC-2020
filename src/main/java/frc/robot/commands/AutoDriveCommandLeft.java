/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

/**
 * An example command that uses an example subsystem.
 */
public class AutoDriveCommandLeft extends CommandBase {
  private SwerveSubsystem robotDrive;
  private ShooterSubsystem robotShooter;
  private Timer autoTimer;
  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public AutoDriveCommandLeft(SwerveSubsystem swerve, ShooterSubsystem shooter) {
    robotDrive = swerve;
    robotShooter = shooter;
    autoTimer = new Timer();
  }

  @Override
  public void initialize() {
    autoTimer.start();
    robotDrive.initDrive();
  }

  @Override
  public void execute() {
    // // Vision Aim
    // do {
    //   robotDrive.updateLimelightTracking(0, 0);
    // } while (!robotDrive.onTarget() || !(autoTimer.hasPeriodPassed(2.0)));

    // // Shoot
    // robotShooter.launchBall(1);
    Timer.delay(2.0);
    // robotShooter.runBallStorage();
    // Timer.delay(4.0);

    // do {
    //   robotDrive.yawToAngle(180.0);
    // } while ((Math.abs(180.0 - robotDrive.gyro.getAngle()) > 1.0) || !(autoTimer.hasPeriodPassed(4.0)));
    //robotDrive.setAllAzimuth(0);
    // robotShooter.runIntake();
    robotDrive.driveSetDistance(1.0);
    //robotDrive.setAllAzimuth(120);
    // do {
    //   robotDrive.yawToAngle(120.0);
    // } while ((Math.abs(120.0 - robotDrive.gyro.getAngle()) > 1.0) || !(autoTimer.hasPeriodPassed(4.0)));
    //robotDrive.driveSetDistance(1.0);
    //robotShooter.launchBall(2);
  }

  @Override
  public boolean isFinished() {
    return true;
  }
}
