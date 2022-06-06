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
    //SWERVE constants
    public static final double swerveModuleXDistance = 0.381;
    public static final double swerveModuleYDistance = 0.381;

    public static final double maxSpeed = 12.0;

    //Encoders
    public static final double angleEncoderConversionFactor = 20.00960392;
    public static final double driveEncoderConversionFactor = 1;

    //PID
    public static final Gains anglePID = new Gains(0.005, 0, 0, 0.0, 0.0, -0.5, 0.5, 0);
    public static final Gains anglePIDFast = new Gains(0.005, 0, 0, 0.0, 0.0, -1, 1, 1);
    public static final Gains fastPID = new Gains(0.05, 0.00001, 0.7, 0.0, 0.0, -1, 1, 1);
    public static final int swervePIDSlot = anglePIDFast.kSlot;
    public static final double PIDdiff = 1;

    //JOYSTICK constants
    public static final double deadzone = 0.05;

    //SPARK ids

    // front left steer fixed
    public static final int frontLeftSteer = 13;

    //front left drive fixed
    public static final int frontLeftDrive = 12;
    
    // front right steer fixed
    public static final int frontRightSteer = 4;

    //front right drive fixed
    public static final int frontRightDrive = 1;

    //rear left steer fixed
    public static final int rearLeftSteer = 2;

    // rear left drive fixed
    public static final int rearLeftDrive = 3;

    // rear right steer fixed
    public static final int rearRightSteer = 14;

    // rear right drive fixed
    public static final int rearRightDrive = 15;
}

