/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.robot.commands.driving;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTableValue;

/**
 * An example subsystem. You can replace with me with your own subsystem.
 */
public class drivetrain extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.
  private final WPI_TalonSRX leftMaster = RobotMap.leftdrive1;
  private final WPI_TalonSRX rightMaster = RobotMap.rightdrive1;
  private final CANSparkMax set1 = RobotMap.setl;
  private final CANSparkMax set2 = RobotMap.setr;
  private double deadzoneleft = 0.15;
  private double deadzoneright = 0.15;
  //NetworkTableEntry ticks;
  //public static NetworkTableInstance networkTableInstance = NetworkTableInstance.getDefault();
  //public static edu.wpi.first.networktables.NetworkTable networkTable;
  public double starttime;


  @Override
  protected void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
    setDefaultCommand(new driving());
    
  }
  public void shoot(){
    set1.setOpenLoopRampRate(1);
    set1.set(1);
    set1.setIdleMode(IdleMode.kCoast);
    set2.setOpenLoopRampRate(1);
    set2.set(1);
    set2.setIdleMode(IdleMode.kCoast);
    if(Robot.m_oi.primaryController.getRawAxis(3)!=0){
      set1.set(-0.57);
      set2.set(0.57   );
      }
      else{
        //set1.setVoltage(0);
        set1.set(0);
        //set2.setVoltage(0);
        set2.set(0);
      }
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
  public void outputToSmartDashboard() {
    SmartDashboard.putNumber("TicksL", leftMaster.getSelectedSensorPosition());
    SmartDashboard.putNumber("TicksR", rightMaster.getSelectedSensorPosition());
  }
  public void driveStraight(double speed){
    rightMaster.set(ControlMode.PercentOutput, speed);
    leftMaster.set(ControlMode.PercentOutput, speed);
  }

  //public void findingtiks(){
  //  networkTableInstance.startServer();
  //  NetworkTableInstance inst = NetworkTableInstance.getDefault();
  //  NetworkTable table = inst.getTable("networkTableInstance");
  //  ticks = table.getEntry("ticks");
  //  ticks.setDouble(rightMaster.getSelectedSensorPosition());
  //}
  public void justdrive(double duration){
    starttime = Timer.getFPGATimestamp();
    double desiredTime = starttime + duration;
    while(Timer.getFPGATimestamp() < desiredTime)
    {
      tank();
    }
  }
}
