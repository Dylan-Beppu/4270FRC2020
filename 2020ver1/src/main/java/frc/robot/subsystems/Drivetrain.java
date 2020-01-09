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

import com.revrobotics.CANSparkMax;
import frc.robot.OI;

import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.robot.commands.*;

/**
 * An example subsystem.  You can replace me with your own Subsystem.
 */
public class Drivetrain extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.
   //Right drive
  private final TalonSRX right1 = RobotMap.rightdrive1;
  private final TalonSRX right2 = RobotMap.rightdrive2;
 
  //Left drive
  private final TalonSRX left1 = RobotMap.leftdrive1;
  private final TalonSRX left2 = RobotMap.leftdrive2;

  public double starttime;

  private double deadzoneleft = 0.1;
  private double deadzoneright = 0.1;

  @Override
  public void initDefaultCommand() {
      setDefaultCommand(new Driving());

    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }

  public void tank(Joystick primaryJoystick){
    //deadzoneleft = 0.1;

    if(Math.abs(primaryJoystick.getRawAxis(1)) > deadzoneleft){
      left1.set(ControlMode.PercentOutput, -primaryJoystick.getRawAxis(1));
      left2.set(ControlMode.PercentOutput, -primaryJoystick.getRawAxis(1));
    }
    else{
      left1.set(ControlMode.PercentOutput, 0);
      left2.set(ControlMode.PercentOutput, 0);
    }

    if(Math.abs(primaryJoystick.getRawAxis(5)) > deadzoneright){
      right1.set(ControlMode.PercentOutput, -primaryJoystick.getRawAxis(5));
      right2.set(ControlMode.PercentOutput, -primaryJoystick.getRawAxis(5));

    }
    else{
      right1.set(ControlMode.PercentOutput,0);
      right2.set(ControlMode.PercentOutput,0);
    }
  }
  public void driveStraight(double speed){
    left1.set(ControlMode.PercentOutput, -speed);
    left2.set(ControlMode.PercentOutput, -speed);
    right1.set(ControlMode.PercentOutput, -speed);
    right2.set(ControlMode.PercentOutput, -speed);
     
  }
  
  public void justdrive(double duration){
    starttime = Timer.getFPGATimestamp();
    double desiredTime = starttime + duration;
    while(Timer.getFPGATimestamp() < desiredTime)
    {
      tank(Robot.m_oi.primaryController);
    }
  }  
}
