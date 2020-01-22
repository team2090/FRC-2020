/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;
import frc.robot.Constants;

public class azimuthPIDSubsystem extends PIDSubsystem {
  /**
   * Creates a new azimuthPIDSubsystem.
   */
  private CANSparkMax outputMotor;
  private AnalogInput encoderInput;

  public azimuthPIDSubsystem(double P, double I, double D, CANSparkMax azimuthMotor, int azimuthEncoder) {
    super(new PIDController(P, I, D));
    outputMotor = azimuthMotor;
    encoderInput = new AnalogInput(azimuthEncoder);
    encoderInput.setOversampleBits(4);
    getController().setTolerance(Constants.azimuthTolerance);
    getController().enableContinuousInput(0, 360);
  }

  @Override
  public void useOutput(double output, double setpoint) {
    SmartDashboard.putNumber("Output value PID", output);
    SmartDashboard.putNumber("Setpoint value PID", setpoint);
    SmartDashboard.putBoolean("At Setpoint?", getController().atSetpoint());
    SmartDashboard.putNumber("Error", setpoint - getMeasurement());

    if (Math.abs(Math.abs(getMeasurement() - setpoint) % 180) < 5.0) {
    // if (getController().atSetpoint()) {
      SmartDashboard.putBoolean("ZERO", true);
      outputMotor.set(0);
    } else {
      SmartDashboard.putBoolean("ZERO", false);
      outputMotor.set(output > 0.8 ? 0.8 : output < -0.8 ? -0.8 : output);
    }
  }

  @Override
  public double getMeasurement() {
    // Return the process variable measurement here
    // SmartDashboard.putNumber("Current value PID", encoderInput.getValue() * 360 / 4049);
    // return encoderInput.getValue() * 360 / 4049;

    SmartDashboard.putNumber("Current value PID", encoderInput.getValue() * 360 / 4049);
    return encoderInput.getValue() * 360 / 4049;
  }

}
