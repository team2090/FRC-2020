/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.kauailabs.navx.frc.AHRS;
import frc.robot.subsystems.Wheel;
import frc.robot.Constants;

/**
 * FRC Team 2090's Swerve Drive Code.
 * 
 * <p>Swerve logic and math is based on Team 2767's <a 
 * href="https://github.com/strykeforce/thirdcoast/tree/master/src/main/java/org/strykeforce/thirdcoast/swerve">
 * Third Coast Java Libraries </a>
 * 
 * <p>Derivation of inverse kinematic equations are from Ether's <a
 * href="https://www.chiefdelphi.com/media/papers/2426">Swerve Kinematics and Programming</a>.
 *
 */
public class SwerveSubsystem extends SubsystemBase {
  private static final Wheel[] wheels = new Wheel[4];
  private final double[] ws = new double[4]; 
  private final double[] wa = new double[4]; 
  private boolean isFieldOriented;
  public final AHRS gyro;
  private final double kLengthComponent;
  private final double kWidthComponent;

  /**
   * This constructs the Swerve Subsystem with the navx and given constants 
   * including the ratio of the robot length to width. The field oriented driving
   * mode is set based on whether the gyro is properly connected.
   */
  public SwerveSubsystem() {
    gyro = new AHRS();
    double radius = Math.hypot(Constants.robotLength, Constants.robotWidth);
    kLengthComponent = Constants.robotLength / radius;
    kWidthComponent = Constants.robotWidth / radius;
    setFieldOriented(true);
    //setFieldOriented(gyro != null && gyro.isConnected());
  }

  /**
   * Instantiates the wheels with the correct ports for the drive motor, azimuth motor, encoder, and angle offset.
   * Wheels are an array numbered 0-3 from front to back, with even numbers on the left side when facing forward
   */
  public void generateWheels() {
    wheels[0] = new Wheel(Constants.FRONT_LEFT_ANGLE_MOTOR, Constants.FRONT_LEFT_DRIVE_MOTOR, Constants.FRONT_LEFT_ENCODER, Constants.FRONT_LEFT_OFFSET);
    // wheels[1] = new Wheel(RobotMap.FRONT_RIGHT_ANGLE_MOTOR, RobotMap.FRONT_RIGHT_DRIVE_MOTOR, RobotMap.FRONT_RIGHT_OFFSET);
    // wheels[2] = new Wheel(RobotMap.BACK_LEFT_ANGLE_MOTOR, RobotMap.BACK_LEFT_DRIVE_MOTOR, RobotMap.BACK_LEFT_OFFSET);
    // wheels[3] = new Wheel(RobotMap.BACK_RIGHT_ANGLE_MOTOR, RobotMap.BACK_RIGHT_DRIVE_MOTOR, RobotMap.BACK_RIGHT_OFFSET);
  }

  /**
   * Initialize the wheels and set them to the absolute zero based on the each Wheel's given offset
   */
  public void initWheels() {
    // for (int i = 0; i < 4; i++) {
    //   wheels[i].initWheel();
    //   wheels[i].zero();
    // }
    wheels[0].initWheel();
    wheels[0].zero();
  }

  /**
   * Returns the array of Wheels
   */
  public Wheel[] getWheels() {
    return wheels;
  }

  /**
   * /**
   * Drive the robot in given field-relative direction and with given rotation.
   *
   * @param forward Y-axis movement, from -1.0 (reverse) to 1.0 (forward)
   * @param strafe X-axis movement, from -1.0 (left) to 1.0 (right)
   * @param azimuth robot rotation, from -1.0 (CCW) to 1.0 (CW)
   */
  public void drive(double forward, double strafe, double yaw) {
    /* If the robot is field oriented, the inputed values are modified to 
     * be with respect to the zero of the field on the navx.
    */
    if (isFieldOriented) {
      double angle = gyro.getAngle();
      SmartDashboard.putNumber("Gyro Angle", gyro.getAngle());

      angle += gyro.getRate();
      angle = Math.IEEEremainder(angle, 360.0);

      angle = Math.toRadians(angle);
      final double temp = forward * Math.cos(angle) + strafe * Math.sin(angle);
      strafe = strafe * Math.cos(angle) - forward * Math.sin(angle);
      forward = temp;
    }

    final double a = strafe - yaw * kLengthComponent;
    final double b = strafe + yaw * kLengthComponent;
    final double c = forward - yaw * kWidthComponent;
    final double d = forward + yaw * kWidthComponent;

    // wheel speed
    ws[0] = Math.hypot(b, d);
    ws[1] = Math.hypot(b, c);
    ws[2] = Math.hypot(a, d);
    ws[3] = Math.hypot(a, c);

    // wheel yaw
    wa[0] = Math.atan2(b, d) * 0.5 / Math.PI;
    wa[1] = Math.atan2(b, c) * 0.5 / Math.PI;
    wa[2] = Math.atan2(a, d) * 0.5 / Math.PI;
    wa[3] = Math.atan2(a, c) * 0.5 / Math.PI;

    // normalize wheel speed
    // final double maxWheelSpeed = Math.max(Math.max(ws[0], ws[1]), Math.max(ws[2], ws[3]));
    // if (maxWheelSpeed > 1.0) {
    //   for (int i = 0; i < 4; i++) {
    //     ws[i] /= maxWheelSpeed;
    //   }
    // }
    // set wheels
    // for (int i = 0; i < 4; i++) {
    //   wheels[i].set(wa[i], ws[i]);
    // }
    wheels[0].set(wa[0], ws[0]);
  }

  /**
   * 
   * Set the drive with respect to the field
   *
   * @param enable true to set robot to field oriented driving, false to set to robot oriented
   */
  public void setFieldOriented(boolean enable) {
    isFieldOriented = enable;
  }

  /**
   * Stop all Swerve Modules
   */
  public void stop() {
    // for (Wheel wheel : wheels) {
    //   wheel.stop();
    // }
    wheels[0].stop();
    SmartDashboard.putString("State", "Stopped");
  }

  /**
   * Change current output based on the SubsystemStateMachine
   */
  public void modifyOutput() {
    
  }
}
