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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.util.Units;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.concurrent.TimeUnit;

import javax.sound.sampled.SourceDataLine;

import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.constraint.SwerveDriveKinematicsConstraint;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;


import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.SparkMaxPIDController;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Gains;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.SwerveModule;

public class SwerveSubsystem extends SubsystemBase {
  private final SwerveModule frontLeft = new SwerveModule(Constants.frontLeftDrive, Constants.frontLeftSteer, 0,true, true,0,false, false);
  private final SwerveModule frontRight = new SwerveModule(Constants.frontRightDrive, Constants.frontRightSteer,0,true,true,0,false, false);
  private final SwerveModule backLeft = new SwerveModule(Constants.rearLeftDrive, Constants.rearLeftSteer,0,true,true,0,false, false);
  private final SwerveModule backRight = new SwerveModule(Constants.rearRightDrive, Constants.rearRightSteer,0,false,true,0,false, false); 
  private final Joystick transJoystick;
  private final Joystick rotJoystick;

  private SlewRateLimiter xLimiter;
  private SlewRateLimiter yLimiter;
  private SlewRateLimiter turningLimiter;
  private SwerveDriveKinematics m_kinematics;
  private PIDController rotPID;

  AHRS navx = new AHRS(Port.kMXP);

  public SwerveSubsystem() {
    transJoystick = new Joystick(Constants.transJoystickPort);
    rotJoystick = new Joystick(Constants.rotJoystickPort);
    m_kinematics = new SwerveDriveKinematics(
      new Translation2d(Constants.kWheelBase / 2, -Constants.kTrackWidth / 2),
      new Translation2d(Constants.kWheelBase / 2, Constants.kTrackWidth / 2),
      new Translation2d(-Constants.kWheelBase / 2, -Constants.kTrackWidth / 2),
      new Translation2d(-Constants.kWheelBase / 2, Constants.kTrackWidth / 2));
    xLimiter = new SlewRateLimiter(Constants.kTeleDriveMaxAccelerationUnitsPerSecond);
    yLimiter = new SlewRateLimiter(Constants.kTeleDriveMaxAccelerationUnitsPerSecond);
    turningLimiter = new SlewRateLimiter(Constants.kTeleDriveMaxAngularAccelerationUnitsPerSecond);
    rotPID = new PIDController(0.05,0,0);
    rotPID.enableContinuousInput(-Math.PI,Math.PI);
    frontLeft.resetEncoders();
    frontRight.resetEncoders();
    backLeft.resetEncoders();
    backRight.resetEncoders();

    SmartDashboard.putNumber("p", 0);
    SmartDashboard.putNumber("i", 0);
    SmartDashboard.putNumber("d", 0);
  }
  

  @Override
  public void periodic() {
    double x= transJoystick.getX();
    double y = transJoystick.getY();
    double rot = rotJoystick.getX();
    boolean setPoimt = transJoystick.getRawButton(1);
    if (transJoystick.getRawButton(5)){
      Constants.tuningSetpoint+=Math.PI/2;
    }else if(transJoystick.getRawButton(4)){
      Constants.tuningSetpoint-=Math.PI/2;
    }
    SmartDashboard.putNumber("setPointReal", Constants.tuningSetpoint);

    x = Math.abs(x) > 0.15 ? x : 0.0;
    y = Math.abs(y) > 0.15 ? y : 0.0;
    rot = Math.abs(rot) > 0.05 ? rot : 0.0;
    
    // 3. Make the driving smoother
    x = xLimiter.calculate(x) * Constants.kTeleDriveMaxSpeedMetersPerSecond;
    y = yLimiter.calculate(y) * Constants.kTeleDriveMaxSpeedMetersPerSecond;
    rot= turningLimiter.calculate(rot)
            * Constants.kTeleDriveMaxAngularSpeedRadiansPerSecond;
    ChassisSpeeds chassisSpeeds1 = new ChassisSpeeds(y,x, rot);
    SwerveModuleState[] moduleStates = m_kinematics.toSwerveModuleStates(chassisSpeeds1);
    setAllPIDControllers(SmartDashboard.getNumber("p",0.05), SmartDashboard.getNumber("i", 0), SmartDashboard.getNumber("d", 0));
  
    SmartDashboard.putNumber("JOYSTICK Y", y);
    this.setModuleStates(moduleStates);
    
    SmartDashboard.putNumber("Module1ROT", moduleStates[0].angle.getRadians());
    SmartDashboard.putNumber("Module2ROT", moduleStates[1].angle.getRadians());
    SmartDashboard.putNumber("Module3ROT", moduleStates[2].angle.getRadians());
    SmartDashboard.putNumber("Module4ROT", moduleStates[3].angle.getRadians());
    SmartDashboard.putNumber("Module1CurrentROT",frontLeft.getRotPosition());
    SmartDashboard.putNumber("Module2CurrentROT", frontRight.getRotPosition());
    SmartDashboard.putNumber("Module3CurrentROT", backLeft.getRotPosition());
    SmartDashboard.putNumber("Module4CurrentROT", backRight.getRotPosition());

    SmartDashboard.putNumber("ChassisSpeeds POT", chassisSpeeds1.omegaRadiansPerSecond);
    SmartDashboard.putNumber("ChassisSpeed X", chassisSpeeds1.vxMetersPerSecond);
    SmartDashboard.putNumber("ChassisSpeed Y", chassisSpeeds1.vyMetersPerSecond);


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
    
    SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.kTeleDriveMaxSpeedMetersPerSecond);
    frontLeft.setDesiredState(desiredStates[0]);
    frontRight.setDesiredState(desiredStates[1]);
    backLeft.setDesiredState(desiredStates[2]);
    backRight.setDesiredState(desiredStates[3]);
}
private void setAllPIDControllers(double p, double i, double d) {
  frontRight.setPidController(p, i, d);
  frontLeft.setPidController(p, i, d);
  backRight.setPidController(p, i, d);
  backLeft.setPidController(p, i, d);
}

}
