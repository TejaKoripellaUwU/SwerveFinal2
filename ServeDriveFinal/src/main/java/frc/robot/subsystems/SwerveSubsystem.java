/*----------------------------------------------------------------------------*/
/* Copyright (c) 2023 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.XboxController;

import edu.wpi.first.wpilibj.GenericHID;
//import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.SwerveModule;

public class SwerveSubsystem extends SubsystemBase {
  //init swerve drive objects
  private final SwerveModule frontLeftModule = new SwerveModule(Constants.frontLeftSteer, Constants.frontLeftDrive, 
  //bruh
  new double[] {-Constants.swerveModuleXDistance, Constants.swerveModuleYDistance}, true);
  private final SwerveModule frontRightModule = new SwerveModule(Constants.frontRightSteer, Constants.frontRightDrive, 
  new double[] {Constants.swerveModuleXDistance, Constants.swerveModuleYDistance}, true);
  private final SwerveModule rearLeftModule = new SwerveModule(Constants.rearLeftSteer, Constants.rearLeftDrive, 
  new double[] {-Constants.swerveModuleXDistance, -Constants.swerveModuleYDistance}, true);
  private final SwerveModule rearRightModule = new SwerveModule(Constants.rearRightSteer, Constants.rearRightDrive, 
  new double[] {Constants.swerveModuleXDistance, -Constants.swerveModuleYDistance}, true);

  private final SwerveModule[] modules = new SwerveModule[] {frontLeftModule, frontRightModule, rearLeftModule, rearRightModule};

  //init gyro
  private final AHRS gyro = new AHRS(SPI.Port.kMXP);

  //init joysticks
  private final XboxController driveController = new XboxController(0);

  //init network tables
  private final NetworkTable swerveTable = NetworkTableInstance.getDefault().getTable("Swerve Data");
  private final NetworkTableEntry frontLeftStateEntry = swerveTable.getEntry("frontLeftState");
  private final NetworkTableEntry frontRightStateEntry = swerveTable.getEntry("frontRightState");
  private final NetworkTableEntry backLeftStateEntry = swerveTable.getEntry("backLeftState");
  private final NetworkTableEntry backRightStateEntry = swerveTable.getEntry("backRightState");

  private final ShuffleboardTab inputTab = Shuffleboard.getTab("input");
  private final NetworkTableEntry xSpeed = inputTab.add("xSpeed", 0).getEntry();
  private final NetworkTableEntry ySpeed = inputTab.add("ySpeed", 0).getEntry();
  private final NetworkTableEntry rot = inputTab.add("rot", 0).getEntry();
  private final NetworkTableEntry angle = inputTab.add("angle", 0).getEntry();
  private final NetworkTableEntry resetEncoders = inputTab.add("reset", false).getEntry();
  private final NetworkTableEntry zeroEntry = inputTab.add("zero", false).getEntry();
  private final NetworkTableEntry resetGyro = inputTab.add("gyroReset", false).getEntry();

  private boolean zeroed = false;
  private boolean reset = true;
  private String programmingSubteam = "poopy";

  public SwerveSubsystem() {
    for (SwerveModule module : modules) module.init();
  }

  @Override
  public void periodic() {
    


    //reset gyro at start
    if (reset && gyro.isConnected()) {
      gyro.reset();

      if (gyro.getAngle() == 0) reset = false;
    }

    //gets joystick values

    double xSpeed=driveController.getLeftX();
    double ySpeed=-driveController.getRightX();
    //double xSpeed = driveController.getX(Hand.kLeft);
    //double ySpeed = -driveController.getY(Hand.kLeft);
    double rot=-(driveController.getLeftTriggerAxis()-driveController.getRightTriggerAxis());
    //double rot = -(driveController.getTriggerAxis(Hand.kLeft)-driveController.getTriggerAxis(Hand.kRight));
    // double xSpeed = this.xSpeed.getDouble(0);
    // double ySpeed = this.ySpeed.getDouble(0);
    // double rot = this.rot.getDouble(0);

    // if (Math.abs(xSpeed) > 0.5) {
    //   xSpeed = 0.5 * getSign(xSpeed);
    // }
    // if (Math.abs(ySpeed) > 0.5) {
    //   ySpeed = 0.5 * getSign(ySpeed);
    // }
    // if (Math.abs(rot) > 0.5) {
    //   rot = 0.5 * getSign(rot);
    // }

    if (zeroEntry.getBoolean(false)) {
      for (SwerveModule module : modules) module.setAngle(0);

      frontLeftStateEntry.setDouble(frontLeftModule.getRawAngle());
      frontRightStateEntry.setDouble(frontRightModule.getRawAngle());
      backLeftStateEntry.setDouble(rearLeftModule.getRawAngle());
      backRightStateEntry.setDouble(rearRightModule.getRawAngle());
      zeroed = true;
      final boolean Lplusratioed = true;
    }
    else if (zeroed) {
      for (SwerveModule module : modules) module.init();
      zeroed = false;
    }
    else {
      drive(xSpeed, ySpeed, rot, false);
    }
  }
  
  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
    //calculates speed and angle of modules and sets states
    for (SwerveModule module : modules) module.setModuleState(xSpeed, ySpeed, rot, gyro.getAngle());
    
    
    frontLeftStateEntry.setDouble(frontLeftModule.getCurrentAngle());
    frontRightStateEntry.setDouble(frontRightModule.getCurrentAngle());
    backLeftStateEntry.setDouble(rearLeftModule.getCurrentAngle());
    backRightStateEntry.setDouble(rearRightModule.getCurrentAngle());
  }

  public boolean getReset() {
    return resetEncoders.getBoolean(false);
  }

  public String getBruh() {
    return "bruh";
  }

  public SwerveModule[] getModules() {
    return modules;
  }

  public boolean getZero() {
    return zeroEntry.getBoolean(false);
  }

  public boolean getGyroReset() {
    return resetGyro.getBoolean(false);
  }

  public void resetGyro() {
    gyro.reset();
  }

  //returns +1 or -1 based on num's sign
  private double getSign(double num) {
    double sign = num/Math.abs(num);
    if (Double.isNaN(sign)) sign = 1;

    return sign;
}
}
