/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.HangConstants.*;

public class HangSubsystem extends SubsystemBase {
  private DoubleSolenoid arm;
  // private Servo lock1;
  // private Servo lock2;
  
  /**
   * Creates a new HangSubsystem.
   */
  public HangSubsystem() {
    arm = new DoubleSolenoid(armForwardChannel, armReverseChannel);
    // lock1 = new Servo(lock1port);
    // lock2 = new Servo(lock2port);
    arm.set(DoubleSolenoid.Value.kReverse);
  }

  public void lift() {
    arm.set(DoubleSolenoid.Value.kForward);
  }

  public void defaultPosition() {
    arm.set(DoubleSolenoid.Value.kReverse);
  }
}
