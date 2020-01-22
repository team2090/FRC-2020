/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants.  This class should not be used for any other purpose.  All constants should be
 * declared globally (i.e. public static).  Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

  // Front Left
  public static int FRONT_LEFT_ANGLE_MOTOR = 0;
  public static int FRONT_LEFT_DRIVE_MOTOR = 10;
  public static double FRONT_LEFT_OFFSET = 0;
  public static int FRONT_LEFT_ENCODER = 1;

  // Front Right
  public static int FRONT_RIGHT_ANGLE_MOTOR = 1;
  public static int FRONT_RIGHT_DRIVE_MOTOR = 11;
  public static double FRONT_RIGHT_OFFSET = 0;
  public static int FRONT_RIGHT_ENCODER = 1;

  // Back Right
  public static int BACK_RIGHT_ANGLE_MOTOR = 3;
  public static int BACK_RIGHT_DRIVE_MOTOR = 13;
  public static double BACK_RIGHT_OFFSET = 0;
  public static int BACK_RIGHT_ENCODER = 2;
  
  // Back Left
  public static int BACK_LEFT_DRIVE_MOTOR = 12;
  public static int BACK_LEFT_ANGLE_MOTOR = 2;
  public static double BACK_LEFT_OFFSET = 0;
  public static int BACK_LEFT_ENCODER = 3;

  // Azimuth PID coefficients
  public static double azimuthkP = 5e-4;
  public static double azimuthkI = 0;
  public static double azimuthkD = 0;
  public static double azimuthkF = 0.0;
  public static double azimuthTolerance = 5.0;

  // PID coefficients
  public static double drivekP = 5e-5; 
  public static double drivekI = 1e-6;
  public static double drivekD = 0; 
  public static double drivekIz = 0; 
  public static double drivekFF = 0.000156; 
  public static double drivekMaxOutput = 1; 
  public static double drivekMinOutput = -1;
  public static double driveMaxRPM = 5700;

  // Smart Motion Coefficients
  public static double maxVel = 2500; // rpm
  public static double maxAcc = 1500;
}
