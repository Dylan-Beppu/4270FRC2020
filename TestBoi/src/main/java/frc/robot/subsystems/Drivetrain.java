/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

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
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.BaseTalonPIDSetConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonSRXConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonSRXPIDSetConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
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
  //gear ration goes form motor-> encoder -> first stage 10:1 -> second stage 12:1
  //first stage 10 to 1
  //second stage 12 to 1

  private static final double kWheelRadiusInches = 2.0;

   //Right drive
  private final WPI_TalonSRX rightMaster = RobotMap.rightdrive1;
  //private final TalonSRX rightSub = RobotMap.rightdrive2;
  //private final CANSparkMax rightMaster = RobotMap.rightdrive1;

 
  //Left drive
  private final WPI_TalonSRX leftMaster = RobotMap.leftdrive1;
  //private final TalonSRX leftSub = RobotMap.leftdrive1;
  //private final CANSparkMax leftMaster = RobotMap.leftdrive1;
  //private final CANEncoder leftCanEncoder = RobotMap.Leftencoder

  //private double ticksPerMeater = 213649;
  private double deadzoneleft = 0.1;
  private double deadzoneright = 0.1;
  
  private AHRS Gyro = RobotMap.gyro;

  //DifferentialDriveKinematics kinematics2 = new DifferentialDriveKinematics(Units.inchesToMeters(28)); //inches between distance between wheels
  DifferentialDriveKinematics kinematics = new DifferentialDriveKinematics(Units.inchesToMeters(24));
  
  //DifferentialDriveOdometry odometry2 = new DifferentialDriveOdometry(kinematics2, getHeading());
  DifferentialDriveOdometry odometry = new DifferentialDriveOdometry(getHeading());

  SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(0.3, 1.96, 0.06); 

  
  PIDController leftPIDController = new PIDController(1, 0, 0);
  PIDController rightPIDController = new PIDController(1, 0, 0);
  //TalonSRXConfiguration leftPIDController = new TalonSRXConfiguration().primaryPID.;
  //BaseTalonPIDSetConfiguration(QuadEncoder);
  //TalonSRXPIDSetConfiguration leftPIDController = new TalonSRXPIDSetConfiguration();
  //configs.primaryPID.selectedFeedbackSensor = TalonSRXFeedbackDevice.QuadEncoder.toFeedbackDevice();
  //configs.primaryPID.selectedFeedbackSensor = TalonFXFeedbackDevice.IntegratedSensor.toFeedbackDevice();

  //TalonSRXPIDSetConfiguration leftPIDController = new TalonSRXPIDSetConfiguration();  
  //TalonSRXPIDSetConfiguration rightPIDController = new TalonSRXPIDSetConfiguration();


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

  public void tank(){
    if(Math.abs(Robot.m_oi.primaryController.getRawAxis(1)) > deadzoneleft){
      leftMaster.set(ControlMode.PercentOutput, Robot.m_oi.primaryController.getRawAxis(1));
    }
    else{
      leftMaster.set(ControlMode.PercentOutput, 0);
    }

    if(Math.abs(Robot.m_oi.primaryController.getRawAxis(5)) > deadzoneright){
      rightMaster.set(ControlMode.PercentOutput, Robot.m_oi.primaryController.getRawAxis(5));
    }
    else{
      rightMaster.set(ControlMode.PercentOutput,0);
    }
  }

  public Rotation2d getHeading() {
    return Rotation2d.fromDegrees(-Gyro.getAngle());
    
  }
  //leftMaster.getActiveTrajectoryVelocity()
      
  public DifferentialDriveWheelSpeeds getSpeeds() {
    return  new DifferentialDriveWheelSpeeds(
        -leftMaster.getSelectedSensorVelocity()*100 /1024 *60 / kGearRatio * 2 * Math.PI * Units.inchesToMeters(kWheelRadiusInches) / 60,  
        rightMaster.getSelectedSensorVelocity()*100 /1024 *60 / kGearRatio * 2 * Math.PI * Units.inchesToMeters(kWheelRadiusInches) / 60);
  }
  public double getLdistance(){
    return -leftMaster.getSelectedSensorPosition()*100 / 201865;
  }
  //ltixks = 225433
  //rticks = 201865
  public double getRdistance(){
    return rightMaster.getSelectedSensorPosition()*100 / 225433;
  }
  
  public void resetpos(){
    leftMaster.setSelectedSensorPosition(0);
    rightMaster.setSelectedSensorPosition(0);
  }

  public double getLvelocity(){
    //leftMaster
    //return -leftMaster.getSelectedSensorVelocity()*100 / 201865;
    return leftMaster.getSelectedSensorVelocity()*100 /1024 *60 / kGearRatio * 2 * Math.PI * Units.inchesToMeters(kWheelRadiusInches) / 60;
  }
//19.71337890625‬ rotstions to m
  public double getRvelocity(){
    //return rightMaster.getSelectedSensorVelocity()*100 / 225433;
    return rightMaster.getSelectedSensorVelocity()*100 /1024 *60 / kGearRatio * 2 * Math.PI * Units.inchesToMeters(kWheelRadiusInches) / 60;

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
    //rightMaster.configVoltageCompSaturation(rightVolts / 12);
    rightMaster.set(rightVolts / 12);
  }

  public void reset() {
    odometry.resetPosition(new Pose2d(), getHeading());
  }

  @Override
  public void periodic() {
    pose = odometry.update(getHeading(), getLvelocity(), getRvelocity());
  }



  
}
