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

  public final JoystickButton robotOrientedForward = new JoystickButton(joystick1, 3);
  public final JoystickButton robotOrientedLeft = new JoystickButton(joystick1, 4);
  public final JoystickButton robotOrientedRight = new JoystickButton(joystick1, 5);
  public final JoystickButton zeroAzimuthPosition = new JoystickButton(joystick1, 8);
  public final JoystickButton slowDriveMode = new JoystickButton(joystick1, 10);
  public final JoystickButton normalDriveMode = new JoystickButton(joystick1, 11);
  public final JoystickButton setFieldOriented = new JoystickButton(joystick2, 6);
  public final JoystickButton setRobotOriented = new JoystickButton(joystick2, 7);

    public double getForward() {
      return -joystick1.getRawAxis(1);
    }
  
    /** Left stick Y (left-right) axis. */
    public double getStrafe() {
      return joystick1.getRawAxis(0);
    }
  
    /** Right stick Y (left-right) axis. */
    public double getYaw() {
      return joystick2.getRawAxis(0);
    }
}
