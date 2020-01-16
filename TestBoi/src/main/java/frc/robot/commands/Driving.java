/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import frc.robot.subsystems.Drivetrain;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.VictorSP;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.OI;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.networktables.*;
import java.math.*;


/**
 * An example command.  You can replace me with your own command.
 */
public class Driving extends Command {
  double HaveTarget = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tv").getDouble(0);
  double Xtar = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0);
  double Ytar = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ty").getDouble(0);
  double TarArea = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ta").getDouble(0);

  private double CamHight = 0; //camra hight from grond in CM
  private double TarHight1 = 1; //target hight in cm
  private double TarHight2 = 1; //target hight in cm
  private double DTT; //Distance to target

  public Driving() {
    // Use requires() here to declare subsystem dependencies
    requires(Robot.kDrivetrain);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    if(Robot.m_oi.primaryController.getRawButtonPressed(5)){
      double DTT = (TarHight1-CamHight) / Math.tan(Xtar);
      //vision if statment, need distance
     if(DTT >= 15 ){
       
     }
     
    }
    else{
      Robot.kDrivetrain.tank(Robot.m_oi.primaryController);
    }
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return false;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    Robot.kDrivetrain.driveStraight(0);
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
  }

}

