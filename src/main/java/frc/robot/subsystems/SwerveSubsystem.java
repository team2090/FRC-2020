/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.kauailabs.navx.frc.AHRS;
import static frc.robot.Constants.SwerveConstants.*;
import static frc.robot.Constants.LimelightConstants.*;

/**
 * FRC Team 2090's Swerve Drive Code.
 * 
 * <p>Swerve logic and math is based on Team 2767's <a 
 * href="https://github.com/strykeforce/thirdcoast/">Third Coast Java Libraries </a>
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
  private final NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");

  /**
   * This constructs the Swerve Subsystem with the navx and given constants 
   * including the ratio of the robot length to width. 
   */
  public SwerveSubsystem() {
    gyro = new AHRS();
    double radius = Math.hypot(robotLength, robotWidth);
    kLengthComponent = robotLength / radius;
    kWidthComponent = robotWidth / radius;
    setFieldOriented(true);
    generateWheels();
  }

  /**
   * Instantiates the wheels with the given ports for the drive motor, azimuth motor, encoder, and angle offset.
   * Wheels are an array numbered 0-3 from front to back, with even numbers on the left side when facing forward.
   * The wheels are initialized (setting up PID) and zeroed based on the offset.
   */
  public void generateWheels() {
    wheels[0] = new Wheel(FRONT_LEFT_ANGLE_MOTOR, FRONT_LEFT_DRIVE_MOTOR, FRONT_LEFT_ENCODER, FRONT_LEFT_OFFSET);
    wheels[1] = new Wheel(FRONT_RIGHT_ANGLE_MOTOR, FRONT_RIGHT_DRIVE_MOTOR, FRONT_RIGHT_ENCODER, FRONT_RIGHT_OFFSET);
    wheels[2] = new Wheel(BACK_LEFT_ANGLE_MOTOR, BACK_LEFT_DRIVE_MOTOR, BACK_LEFT_ENCODER, BACK_LEFT_OFFSET);
    wheels[3] = new Wheel(BACK_RIGHT_ANGLE_MOTOR, BACK_RIGHT_DRIVE_MOTOR, BACK_RIGHT_ENCODER, BACK_RIGHT_OFFSET);
    
    for (Wheel wheel : wheels) {
      wheel.initWheel();
      wheel.zero();
    }
  }

  /**
   * Returns an array of Wheels
   */
  public Wheel[] getWheels() {
    return wheels;
  }

  /**
   * 
   * Drive the robot in given field-relative direction and with given rotation.
   *
   * @param forward Y-axis movement, from -1.0 (reverse) to 1.0 (forward)
   * @param strafe X-axis movement, from -1.0 (left) to 1.0 (right)
   * @param azimuth robot rotation, from -1.0 (CCW) to 1.0 (CW)
   */
  public void drive(double forward, double strafe, double yaw) {
    isFieldOriented = SmartDashboard.getBoolean("driveMode", true);
    /* If the robot is field oriented, the inputed values are modified to 
     * be with respect to the zero of the field on the navx.
    */
    if (isFieldOriented) {
      double angle = gyro.getAngle();
      SmartDashboard.putNumber("Gyro", gyro.getAngle());

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
    final double maxWheelSpeed = Math.max(Math.max(ws[0], ws[1]), Math.max(ws[2], ws[3]));
    if (maxWheelSpeed > 1.0) {
      for (int i = 0; i < 4; i++) {
        ws[i] /= maxWheelSpeed;
      }
    }
    // set wheels
    for (int i = 0; i < 4; i++) {
      wheels[i].set(wa[i], ws[i]);
    }   
  }

  /**
   * Set the drive mode for the robot
   *
   * @param enable true = field oriented driving, false = robot oriented
   */
  public void setFieldOriented(boolean enable) {
    isFieldOriented = enable;
    SmartDashboard.putBoolean("driveMode", enable);
  }

  /**
   * Set the wheels at a given target angle
   * @param angle the target angle for all swerve modules
   */
  public void setAllAzimuth(double angle) {
    for (Wheel wheel : wheels) {
      wheel.setTargetAngle(angle);
    }
  }

  /**
   * Drive a set distance
   * @param distance the target distance for the modules to drive
   */
  public void driveSetDistance(double distance) {
    for (Wheel wheel : wheels) {
      wheel.setTargetDistance(distance);
    }
  }

  /**
   * Stop all Swerve Modules
   */
  public void stop() {
    for (Wheel wheel : wheels) {
      wheel.stop();
    }
  }

  /**
   * Init all Swerve Modules in the zeroed position (azimuth position and drive output are set to 0)
   * Camera mode is set to normal (not vision)
   */
  public void initDrive() {
    gyro.zeroYaw();
    for (Wheel wheel : wheels) {
      wheel.zero();
    }
    setCamMode(false);
  }

  /**
   * Aiming at the target by modifying yaw input 
   * @param forward Y-axis movement, from -1.0 (reverse) to 1.0 (forward)
   * @param strafe X-axis movement, from -1.0 (left) to 1.0 (right)
   */
  public void updateLimelightTracking(double forward, double strafe) {
    setCamMode(true);
    double headingError = table.getEntry("tx").getDouble(0);
    double yawInput = 0;
    yawInput = Math.abs(headingError) < maxHeadingError ? 0 : (headingError * headingConstant);
    yawInput = Math.abs(yawInput) > maxYawOutput ? Math.copySign(maxYawOutput, yawInput): yawInput;
    drive(forward, strafe, yawInput);
  }

  /**
   * Set the limelight's camera mode
   * @param visionMode true = vision, false = normal viewing
   */
  public void setCamMode(boolean visionMode) {
    table.getEntry("camMode").setNumber(visionMode ? 0 : 1);
  }

  /**
   * Check if the swerve is currently aimed at the target
   */
  public boolean onTarget() {
    setCamMode(true);
    double headingError = table.getEntry("tx").getDouble(0);
    setCamMode(false);
    return Math.abs(headingError) < 2.0 ? true : false;
  }

  /**
   * Yaw the robot to a desired orientation
   * @param targetPosition the position that the robot will face
   */
  public void yawToAngle(double targetPosition) {
    drive(0, 0, headingConstant * (targetPosition - gyro.getAngle()));
  }
}
