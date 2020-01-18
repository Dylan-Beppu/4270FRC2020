/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Talon;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
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
//import 


/**
 * An example subsystem.  You can replace me with your own Subsystem.
 */
public class Drivetrain extends SubsystemBase {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.
   //Right drive
  private final TalonSRX rightMaster = RobotMap.rightdrive1;
  private final TalonSRX rightSub = RobotMap.rightdrive2;

 
  //Left drive
  private final TalonSRX leftMaster = RobotMap.leftdrive1;
  private final TalonSRX leftSub = RobotMap.leftdrive1;

  public Drivetrain() {
    leftSub.follow(leftMaster);
    rightSub.follow(rightMaster);

    leftMaster.setInverted(false);
    rightMaster.setInverted(true);
  }



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


}
