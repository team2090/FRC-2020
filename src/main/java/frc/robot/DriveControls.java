/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;
import edu.wpi.first.wpilibj.Joystick;
/**
 * Add your docs here.
 */
public class DriveControls {
    private final Joystick joystick1 = new Joystick(0);
    private final Joystick joystick2 = new Joystick(1);

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

    public boolean robotOriented() {
      return joystick1.getRawButton(5);
    }

    public boolean fieldOriented() {
      return joystick1.getRawButton(6);
    }

    public boolean slowDriveMode() {
      return joystick1.getRawButton(2);
    }

    public boolean robotOrientedForward() {
      return joystick1.getRawButton(4);
    }
}
