/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
/**
 * Add your docs here.
 */
public class DriveControls {
  
  private final Joystick joystick1 = new Joystick(0);
  private final Joystick joystick2 = new Joystick(1);
  private final Joystick joystick3 = new Joystick(2);

  // Joystick 1
  public final JoystickButton limelightAim = new JoystickButton(joystick1, 1);
  public final JoystickButton robotOrientedForward = new JoystickButton(joystick1, 3);
  public final JoystickButton robotOrientedBackwards = new JoystickButton(joystick1, 2);
  public final JoystickButton robotOrientedLeft = new JoystickButton(joystick1, 4);
  public final JoystickButton robotOrientedRight = new JoystickButton(joystick1, 5);
  public final JoystickButton setFieldOriented = new JoystickButton(joystick1, 10);
  public final JoystickButton setRobotOriented = new JoystickButton(joystick1, 11);

  // Joystick 2
  public final JoystickButton intake = new JoystickButton(joystick2, 1);
  public final JoystickButton backwardsBallStorage = new JoystickButton(joystick2, 2);
  public final JoystickButton ballStorage = new JoystickButton(joystick2, 3);
  public final JoystickButton lowerIntake = new JoystickButton(joystick2, 4);
  public final JoystickButton reverseIntake = new JoystickButton(joystick2, 5);
  public final JoystickButton slowDriveMode = new JoystickButton(joystick2, 6);
  public final JoystickButton normalDriveMode = new JoystickButton(joystick2, 7);
  public final JoystickButton precisionDriveMode = new JoystickButton(joystick2, 8);
  public final JoystickButton zeroField = new JoystickButton(joystick2, 11);
  
  // Joystick 3
  public final JoystickButton shootBallMid = new JoystickButton(joystick3, 3);
  public final JoystickButton shootBallLow = new JoystickButton(joystick3, 4);
  public final JoystickButton shootBallHigh = new JoystickButton(joystick3, 5);
  public final JoystickButton hangDown = new JoystickButton(joystick3, 10);
  public final JoystickButton hangUp = new JoystickButton(joystick3, 11);
  
  public double getForward() {
    return -joystick1.getRawAxis(1);
  }

  public double getStrafe() {
    return joystick1.getRawAxis(0);
  }

  public double getYaw() {
    return joystick2.getRawAxis(0);
  }
}
