/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Subsystem;

import com.revrobotics.CANSparkMax;

import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.robot.commands.*;


/**
 * Add your docs here.
 */

public class drivetrain extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.
  private final CANSparkMax left = RobotMap.leftmasterdrive;
  private final CANSparkMax right = RobotMap.rightmasterdrive;

  private final CANSparkMax right1 = RobotMap.rightdrive1;

  private final CANSparkMax right2 = RobotMap.rightdrive2;
  private final CANSparkMax left1 = RobotMap.leftdrive1;
  private final CANSparkMax left2 = RobotMap.leftdrive2;

  public double starttime;



  @Override
  public void initDefaultCommand() {
    setDefaultCommand(new driving());

    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }
  
  public void tank(Joystick stick){
    if(Robot.kShifter.isfast == true){

    if(Math.abs(stick.getRawAxis(1)) > .17){
      left.set(-stick.getRawAxis(1));
      left1.set(-stick.getRawAxis(1));
      left2.set(-stick.getRawAxis(1));

    }
    else{
      left.set(0);
      left1.set(0);
      left2.set(0);
    }
    if(Math.abs(stick.getRawAxis(5)) > .17){
      right.set(-stick.getRawAxis(5));
      right1.set(-stick.getRawAxis(5));
      right2.set(-stick.getRawAxis(5));

    }
    else{
      right.set(0);
      right1.set(0);
      right2.set(0);
    }
  }
  else{ /// WHEN SLOW
    if(Math.abs(stick.getRawAxis(1)) > .17){
      left.set(-stick.getRawAxis(1) * 0.8);
      left1.set(-stick.getRawAxis(1) * 0.8);
      left2.set(-stick.getRawAxis(1) * 0.8);

    }
    else{
      left.set(0);
      left1.set(0);
      left2.set(0);
    }
    if(Math.abs(stick.getRawAxis(5)) > .17){
      right.set(-stick.getRawAxis(5) * 0.8);
      right1.set(-stick.getRawAxis(5) * 0.8);
      right2.set(-stick.getRawAxis(5) * 0.8);

    }
    else{
      right.set(0);
      right1.set(0);
      right2.set(0);
    }
  }
  }

  public void drivestraight(double speed){
    left.set(-speed);
    left1.set(-speed);
    left2.set(-speed);
    right.set(-speed);
    right1.set(-speed);
    right2.set(-speed);


  }
  
  public void driveleft(double leftspeed){
    left.set(-leftspeed);
  }
  
  public void driveright(double rightspeed) {
    right.set(-rightspeed);
  }


  public void justdrive(double duration){
    starttime = Timer.getFPGATimestamp();
    double desiredTime = starttime + duration;
    while(Timer.getFPGATimestamp() < desiredTime)
    {
      tank(Robot.m_oi.stick);
    }
  }
}

