/*----------------------------------------------------------------------------*/
/* Copyright (c) 2023 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.SPI.Port;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

import edu.wpi.first.math.util.Units;

import java.util.concurrent.TimeUnit;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;


import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.SwerveModule;

public class SwerveSubsystem extends SubsystemBase {
  private final SwerveModule frontLeft = new SwerveModule(Constants.frontLeftDrive, Constants.frontLeftSteer, 0,false, false,0,false, false);
  private final SwerveModule frontRight = new SwerveModule(Constants.frontRightDrive, Constants.frontRightSteer,0,false,false,0,false, false);
  private final SwerveModule backLeft = new SwerveModule(Constants.rearLeftDrive, Constants.rearLeftSteer,0,false,false,0,false, false);
  private final SwerveModule backRight = new SwerveModule(Constants.rearRightDrive, Constants.rearRightSteer,0,false,false,0,false, false); 
  private final double trackWidth = edu.wpi.first.math.util.Units.inchesToMeters(Constants.wheelBaseX);
  private final double trackLength = Units.inchesToMeters(Constants.wheelBaseY);
  private final Joystick transJoystick;
  private final Joystick rotJoystick;
  private ChassisSpeeds chassisSpeeds;
  private final edu.wpi.first.math.kinematics.SwerveDriveKinematics m_kinematics;
  AHRS navx = new AHRS(Port.kMXP);

  public SwerveSubsystem() {
    transJoystick = new Joystick(Constants.transJoystickPort);
    rotJoystick = new Joystick(Constants.rotJoystickPort);
    m_kinematics = new SwerveDriveKinematics(new Translation2d(trackWidth,-trackLength),
    new Translation2d(trackWidth,trackLength),new Translation2d(-trackWidth,trackLength),
    new Translation2d(-trackWidth,-trackLength));
    
  }

  @Override
  public void periodic() {
    double x= transJoystick.getX();
    double y = transJoystick.getY();
    double rot = rotJoystick.getX();
    System.out.println("x: " + x);
    System.out.println("Y: " + y);
    System.out.println("ROT: " + rot);

    if (Math.abs(x)<Constants.deadzone){
      x = 0;
    }
    if (Math.abs(y)<Constants.deadzone){
      y = 0;
    }
    if(Math.abs(rot)<Constants.deadzone){
      rot = 0;
    }

    //input scaling
    x= x*Constants.maxSpeed;
    y= y*Constants.maxSpeed;
    rot = rot*Constants.maxSpeed;
    chassisSpeeds = new ChassisSpeeds(x, y, rot);
    SwerveModuleState[] moduleStates = m_kinematics.toSwerveModuleStates(chassisSpeeds);
    this.setModuleStates(moduleStates);

  }
  
  public void resetGyro(){
    navx.reset();
  }
  public double getHeading(){
    return Math.IEEEremainder(navx.getAngle(), 360);
  }
  public Rotation2d getRotation2d(){
    return new Rotation2d(getHeading());
  }
  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, 5);
    frontLeft.setDesiredState(desiredStates[0]);
    frontRight.setDesiredState(desiredStates[1]);
    backLeft.setDesiredState(desiredStates[2]);
    backRight.setDesiredState(desiredStates[3]);
}

}
