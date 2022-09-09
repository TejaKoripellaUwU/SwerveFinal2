package frc.robot;
import java.util.concurrent.locks.Condition;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.controller.PIDController;


public class SwerveModule {

    private int m_MotorTransID;
    private int m_MotorRotID;
    private int m_UniversalEncoderID;
    private CANSparkMax transMotor;
    private CANSparkMax rotMotor;
    private RelativeEncoder transEncoder;
    private RelativeEncoder rotEncoder;
    private AnalogInput universalEncoder;
    public PIDController rotPID;
    private Boolean isAbsoluteEncoder;
    private double universalEncoderOffset;
    private Boolean m_transInverted;
    private Boolean m_rotInverted;
    private Boolean encoderInverted;
    private double ye;
   

    public SwerveModule(int motorTransID, int motorRotID, int universalEncoderID,
     Boolean transInverted, Boolean rotInverted, double universalEncoderOffsetinit,
     Boolean universalEncoderInverted, boolean isAbsEncoder){
        this.encoderInverted = universalEncoderInverted;
        this.isAbsoluteEncoder=isAbsEncoder;
        this.m_MotorTransID = motorTransID;
        this.m_UniversalEncoderID = universalEncoderID;
        this.m_MotorRotID = motorRotID;
        this.m_transInverted = transInverted;
        this.m_rotInverted = rotInverted;
        this.universalEncoderOffset = universalEncoderOffsetinit;
        
        transMotor = new CANSparkMax(this.m_MotorTransID, MotorType.kBrushless);
        rotMotor = new CANSparkMax(this.m_MotorRotID, MotorType.kBrushless);
        if (isAbsEncoder){
            universalEncoder = new AnalogInput(this.m_UniversalEncoderID);
        
        }

        transMotor.setInverted(this.m_transInverted);
        rotMotor.setInverted(this.m_rotInverted);

        transEncoder = transMotor.getEncoder();
        rotEncoder = rotMotor.getEncoder();
        
        System.out.println(transEncoder.getPosition());
        rotEncoder.setPositionConversionFactor(2*Math.PI);
        resetEncoders();
        
    }
    public double getTransPosition(){
        
        return transEncoder.getPosition();
    }
    public double getRotPosition(){
        double jesus = rotEncoder.getPosition()- (int)(rotEncoder.getPosition());
        jesus = jesus * 2*Math.PI;
        return jesus;
    
    }
    public double getTransVelocity(){
        return transEncoder.getVelocity();
    }
    public double getRotVelocity(){
        return rotEncoder.getVelocity();
    }
    public double getUniversalEncoderRad(){
        if (isAbsoluteEncoder) {
            double angle = universalEncoder.getVoltage()/RobotController.getVoltage5V();
            angle*=2.0 * Math.PI;
            angle-= universalEncoderOffset;
            if (this.encoderInverted){
                return angle*-1;
            }
            else{
                return angle;
            }
            
        }
        return 0;
        
    }
    public void resetEncoders(){
        transEncoder.setPosition(0);
        rotEncoder.setPosition(0);// 

        //hello - 8/3/22
    }
    public SwerveModuleState getState(){
        return new SwerveModuleState(getTransVelocity(),new Rotation2d(getRotPosition()));
    }
    public void setDesiredState(SwerveModuleState desiredState){
        desiredState = SwerveModuleState.optimize(desiredState, getState().angle);
            SmartDashboard.putNumber("RotationPosition", getRotPosition());
            SmartDashboard.putNumber("DesiredState", desiredState.angle.getRadians());
            transMotor.set(desiredState.speedMetersPerSecond/Constants.maxSpeed);
            if (Constants.tuningPID) {
                rotMotor.set(rotPID.calculate(getRotPosition(), Constants.tuningSetpoint));
            } else {
                rotMotor.set(rotPID.calculate(getRotPosition(),desiredState.angle.getRadians()));
            }
        System.out.println("setPoint is: "+ getRotPosition());


    }
    public void stop() {
        transMotor.set(0);
        rotMotor.set(0);
    }
    public PIDController getPIDController(){
        return this.rotPID;
    }
    public void setPidController(double p, double i, double d){
        rotPID.setP(p);
        rotPID.setI(i);
        rotPID.setD(d);
    }
    

}