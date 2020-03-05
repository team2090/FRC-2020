/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

/**
 * An example command that uses an example subsystem.
 */
public class AutoDriveCommandMid extends AutoDriveCommand {

  private final SwerveSubsystem robotDrive;
  private final ShooterSubsystem robotShooter;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public AutoDriveCommandMid(SwerveSubsystem swerve, ShooterSubsystem shooter) {
    super(swerve, shooter);
    robotDrive = swerve;
    robotShooter = shooter;
  }

  public void autoSequence() {
    
  }
}
