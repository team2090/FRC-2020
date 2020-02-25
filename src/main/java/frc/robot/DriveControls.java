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
    private final Joystick joystick1;
    private final Joystick joystick2;

    public final JoystickButton robotOrientedForward; 
    public final JoystickButton robotOrientedLeft;
    public final JoystickButton robotOrientedRight;
    public final JoystickButton zeroAzimuthPosition;
    public final JoystickButton slowDriveMode;
    public final JoystickButton normalDriveMode;
    public final JoystickButton setFieldOriented;
    public final JoystickButton setRobotOriented;
    
    public DriveControls(int controller1, int controller2) {
      joystick1 = new Joystick(controller1);
      joystick2 = new Joystick(controller2);

      robotOrientedForward = new JoystickButton(joystick1, 3);
      robotOrientedLeft = new JoystickButton(joystick1, 4);
      robotOrientedRight = new JoystickButton(joystick1, 5);
      zeroAzimuthPosition = new JoystickButton(joystick1, 8);
      slowDriveMode = new JoystickButton(joystick1, 10);
      normalDriveMode = new JoystickButton(joystick1, 11);
      setFieldOriented = new JoystickButton(joystick2, 6);
      setRobotOriented = new JoystickButton(joystick2, 7);
    }

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
