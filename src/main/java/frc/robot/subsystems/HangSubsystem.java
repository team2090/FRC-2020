/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.HangConstants.*;

import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class HangSubsystem extends SubsystemBase {
  private DoubleSolenoid arm;
  private CANSparkMax winch1;
  private CANSparkMax winch2;
  private DoubleSolenoid.Value position = Value.kReverse;
  private CANPIDController winchPID;
  
  /**
   * Creates a new HangSubsystem.
   */
  public HangSubsystem() {
    arm = new DoubleSolenoid(armForwardChannel, armReverseChannel);
    arm.set(position);
    
    winch1 = new CANSparkMax(6, MotorType.kBrushless);
    winch2 = new CANSparkMax(7, MotorType.kBrushless);
    winch1.follow(winch2);
    winchPID = winch2.getPIDController();
    winchPID.setP(winchkP);
    winchPID.setI(winchkI);
    winchPID.setD(winchkD);
    winchPID.setIZone(winchkIz);
    winchPID.setFF(winchkFF);
    winchPID.setOutputRange(winchkMinOutput, winchkMaxOutput);
  }

  public void lift() {
    arm.set(DoubleSolenoid.Value.kForward);
    position = DoubleSolenoid.Value.kForward;
  }

  public void down() {
    arm.set(DoubleSolenoid.Value.kReverse);
    position = DoubleSolenoid.Value.kReverse;
  }

  public void holdPosition() {
    arm.set(position);
  }
}
