/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Subsystem;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableValue;

import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.robot.commands.*;

import frc.robot.subsystems.Drivetrain;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.VictorSP;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.networktables.*;
import java.math.*;


/**
 * An example subsystem.  You can replace me with your own Subsystem.
 */
public class Drivetrain extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.
   //Right drive
  private final TalonSRX right1 = RobotMap.rightdrive1;
 
  //Left drive
  private final TalonSRX left1 = RobotMap.leftdrive1;

  public double starttime;

  private double deadzoneleft = 0.1;
  private double deadzoneright = 0.1;

  /*double HaveTarget = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tv").getDouble(0);
  double Xtar = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0);
  double Ytar = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ty").getDouble(0);
  double TarArea = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ta").getDouble(0);

  private double CamHight = 0; //camra hight from grond in CM
  private double TarHight1 = 1; //target hight in cm
  private double TarHight2 = 1; //target hight in cm
  private double DTT;*/

  @Override
  public void initDefaultCommand() {
      setDefaultCommand(new Driving());

    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }

  public void tank(Joystick primaryJoystick){

    if(Math.abs(primaryJoystick.getRawAxis(1)) > deadzoneleft){
      left1.set(ControlMode.PercentOutput, -primaryJoystick.getRawAxis(1));
    }
    else{
      left1.set(ControlMode.PercentOutput, 0);
    }

    if(Math.abs(primaryJoystick.getRawAxis(5)) > deadzoneright){
      right1.set(ControlMode.PercentOutput, -primaryJoystick.getRawAxis(5));
    }
    else{
      right1.set(ControlMode.PercentOutput,0);
    }
  }
  public void driveStraight(double speed){
    left1.set(ControlMode.PercentOutput, -speed);
    right1.set(ControlMode.PercentOutput, -speed);     
  }
  public void LD(double speed){
    left1.set(ControlMode.PercentOutput, -speed);
  }
  public void RD(double speed){
    right1.set(ControlMode.PercentOutput, -speed);
  }

  public void justdrive(double duration){
    starttime = Timer.getFPGATimestamp();
    double desiredTime = starttime + duration;
    while(Timer.getFPGATimestamp() < desiredTime)
    {
      tank(Robot.m_oi.primaryController);
    }
  }
 /* //public final double DTT = (TarHight1-CamHight) / Math.tan(Xtar);
  public void DistantToTarget(){
    DTT = (TarHight1-CamHight) / Math.tan(Xtar);
    
    
    
  }*/
}
