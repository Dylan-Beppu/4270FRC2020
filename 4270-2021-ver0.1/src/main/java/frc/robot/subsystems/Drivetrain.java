package frc.robot.subsystems;

//import edu.wpi.first.wpilibj.Talon;

//import com.ctre.phoenix.motorcontrol.ControlMode;
//import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
//import com.ctre.phoenix.motorcontrol.can.TalonFXPIDSetConfiguration;
//import com.ctre.phoenix.motorcontrol.can.TalonSRX;
//import com.ctre.phoenix.motorcontrol.can.TalonSRXConfiguration;
//import com.kauailabs.*;
//import com.ctre.phoenix.motorcontrol.can.TalonSRXPIDSetConfiguration;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableValue;

import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.robot.commands.*;
import frc.robot.OI;

import frc.robot.subsystems.Drivetrain;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Units; // units class converts imperial to si units

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
//import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.networktables.*;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import java.math.*;
import com.kauailabs.navx.frc.AHRS;
//import 
//import com.revrobotics.CANEncoder;
//import com.revrobotics.CANSparkMax;


/**
 * An example subsystem.  You can replace me with your own Subsystem.
 */
public class Drivetrain extends SubsystemBase {

  private static final double kGearRatio = 22; // gear ratio
  private static final double kWheelRadiusInches = 2.0;

   //Right drive
  private final WPI_TalonFX rightMaster = RobotMap.rightdrive1;
  private final WPI_TalonFX rightSub = RobotMap.rightdrive2;

 
  //Left drive
  private final WPI_TalonFX leftMaster = RobotMap.leftdrive1;
  private final WPI_TalonFX leftSub = RobotMap.leftdrive1;

  //private double ticksPerMeater = 213649;
  private double deadzoneleft = 0.10;
  private double deadzoneright = 0.10;
  
  private AHRS Gyro = RobotMap.gyro;

  //DifferentialDriveKinematics kinematics2 = new DifferentialDriveKinematics(Units.inchesToMeters(28)); //inches between distance between wheels
  DifferentialDriveKinematics kinematics = new DifferentialDriveKinematics(Units.inchesToMeters(24));
  
  //DifferentialDriveOdometry odometry2 = new DifferentialDriveOdometry(kinematics2, getHeading());
  DifferentialDriveOdometry odometry = new DifferentialDriveOdometry(getHeading());

  SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(0.3, 1.96, 0.06); 

  PIDController leftPIDController = new PIDController(1, 0, 0);
  PIDController rightPIDController = new PIDController(1, 0, 0);
  



  Pose2d pose = new Pose2d();

  //TalonSRXPIDSetConfiguration test = new TalonSRXPIDSetConfiguration();

  

  public Drivetrain() {
    //leftSub.follow(leftMaster);
    //rightSub.follow(rightMaster);

    //leftMaster.setInverted(true);
    //rightMaster.setInverted(true);

    //Gryo.reset(); //scrued up spelling again LOL
    Gyro.reset();
  }

  public void lowGear(){
    Robot.kShifter.isfast = false;
  }

  public void highGear(){
    Robot.kShifter.isfast = true;
  }
  
  public void tank(){
    if(Robot.kShifter.isfast == true){
      if(Math.abs(Robot.m_oi.LjoyY) > deadzoneleft){
        leftMaster.set(ControlMode.PercentOutput, Robot.m_oi.LjoyY*0.4);
         leftSub.set(ControlMode.PercentOutput, Robot.m_oi.LjoyY*0.4);

      }
      else{
        leftMaster.set(ControlMode.PercentOutput, 0);
        leftSub.set(ControlMode.PercentOutput, 0);
      }
      if(Math.abs(Robot.m_oi.RjoyY) > deadzoneright){
        rightMaster.set(ControlMode.PercentOutput, Robot.m_oi.RjoyY*0.4);
        rightSub.set(ControlMode.PercentOutput, Robot.m_oi.RjoyY*0.4);
      }
      else{
        rightMaster.set(ControlMode.PercentOutput,0);
        rightSub.set(ControlMode.PercentOutput, 0);
      }
    }
    //when slow
    else{
      if(Math.abs(Robot.m_oi.LjoyY) > deadzoneleft){
        leftMaster.set(ControlMode.PercentOutput, Robot.m_oi.LjoyY);
        leftSub.set(ControlMode.PercentOutput, Robot.m_oi.LjoyY);
      }
      else{
        leftMaster.set(ControlMode.PercentOutput, 0);
        leftSub.set(ControlMode.PercentOutput, 0);
      }
      if(Math.abs(Robot.m_oi.RjoyY) > deadzoneright){
        rightMaster.set(ControlMode.PercentOutput, Robot.m_oi.RjoyY);
        rightSub.set(ControlMode.PercentOutput, Robot.m_oi.RjoyY);      
      }
      else{
        rightMaster.set(ControlMode.PercentOutput,0);
        rightSub.set(ControlMode.PercentOutput, 0);
      } 
    }
  }

  public Rotation2d getHeading() {
    return Rotation2d.fromDegrees(-Gyro.getAngle());
    
  }
  //leftMaster.getActiveTrajectoryVelocity()
      
  public DifferentialDriveWheelSpeeds getSpeeds() {
    return new DifferentialDriveWheelSpeeds(
      leftMaster.getSelectedSensorVelocity()*100 /2048 *60 / kGearRatio * 2 * Math.PI * Units.inchesToMeters(kWheelRadiusInches) / 60 *-1,  
      rightMaster.getSelectedSensorVelocity()*100 /2048 *60 / kGearRatio * 2 * Math.PI * Units.inchesToMeters(kWheelRadiusInches) / 60 *-1);
}
  public double getLdistance(){
    return leftMaster.getSelectedSensorPosition()/2048 /kGearRatio * (Math.PI * Units.inchesToMeters(kWheelRadiusInches) *-1);
  }
  public double getRdistance(){
    return rightMaster.getSelectedSensorPosition()/2048 /kGearRatio * (Math.PI * Units.inchesToMeters(kWheelRadiusInches) *-1);
  }
  

  public double getLvelocity(){

    return leftMaster.getSelectedSensorVelocity()/2048 *60 / kGearRatio * 2 * Math.PI * Units.inchesToMeters(kWheelRadiusInches) / 60 *-1;
    //return leftMaster.getSelectedSensorVelocity()*60 / kGearRatio * 2 * Math.PI * Units.inchesToMeters(kWheelRadiusInches) / 60;
  }

  public double getRvelocity(){
    return rightMaster.getSelectedSensorVelocity()/2048 *60 / kGearRatio * 2 * Math.PI * Units.inchesToMeters(kWheelRadiusInches) / 60 *-1;
    //return rightMaster.getSelectedSensorVelocity()*60 / kGearRatio * 2 * Math.PI * Units.inchesToMeters(kWheelRadiusInches) / 60;

  }

  public DifferentialDriveKinematics getKinematics() {
    return kinematics;
  }

  public Pose2d getPose() {
    return pose;
  }

  public SimpleMotorFeedforward getFeedforward() {
    return feedforward;
  }

  public PIDController getLeftPIDController() {
    return leftPIDController;
  }

  public PIDController getRightPIDController() {
    return rightPIDController;
  }

  public void setOutputVolts(double leftVolts, double rightVolts) {
    //leftMaster.configVoltageCompSaturation(leftVolts / 12);
    leftMaster.set(leftVolts / 12);
    leftSub.set(leftVolts / 12);
    //rightMaster.configVoltageCompSaturation(rightVolts / 12);
    rightMaster.set(rightVolts / 12);
    rightSub.set(rightVolts / 12);    
  }

  public void reset() {
    odometry.resetPosition(new Pose2d(), getHeading());
  }
  public void autoBSpeed(double dirspd){
    RobotMap.rightdrive1.set(-dirspd);
    RobotMap.rightdrive2.set(-dirspd);
    RobotMap.leftdrive1.set(dirspd);
    RobotMap.leftdrive2.set(dirspd);
  }

  @Override
  public void periodic() {
    pose = odometry.update(getHeading(), getLdistance(), getRdistance());
  }
}
