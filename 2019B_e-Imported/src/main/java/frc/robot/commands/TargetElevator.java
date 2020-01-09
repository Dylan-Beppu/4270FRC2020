/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;


import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;

public class TargetElevator extends Command {
  public TargetElevator() {
    requires(Robot.kElevator);
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
//  /*
//low ball
    if(Robot.m_oi.balllow.get() == true && Robot.m_oi.ballmid.get() == false && Robot.m_oi.ballhigh.get() == false && Robot.m_oi.hatchlow.get() == false && Robot.m_oi.hatchmid.get() == false && Robot.m_oi.hatchhigh.get() == false ){
      if(Robot.kElevator.intUP == false){
      Robot.kElevator.complexElevate(50, 1);
      Robot.kElevator.armdown();
      }
      else{
        Robot.kElevator.targetIntake(31, .4);
      //  Robot.kDrivetrain.justdrive(1.3);
        Robot.kElevator.complexElevate(50, 1);
      Robot.kElevator.armdown();
      }
      Robot.kDrivetrain.justdrive(.5);
      Robot.kElevator.targetIntake(4, .6);
      ////Robot.kElevator.intakeup();

    }
//mid ball
    else if(Robot.m_oi.balllow.get() == false && Robot.m_oi.ballmid.get() == true && Robot.m_oi.ballhigh.get() == false && Robot.m_oi.hatchlow.get() == false && Robot.m_oi.hatchmid.get() == false && Robot.m_oi.hatchhigh.get() == false){
      Robot.kElevator.doneyet = false;
      if(Robot.kElevator.intUP == false){
        Robot.kElevator.targetIntake(38, .4);
        Robot.kElevator.doneyet = false;
      while(Robot.kElevator.doneyet != true){
      Robot.kElevator.complexElevate(20, 1); //14 before
      }
      Robot.kElevator.armup();
      }
      else{
        Robot.kElevator.targetIntake(38, .4);
     //   Robot.kDrivetrain.justdrive(1.3);
        Robot.kElevator.doneyet = false;
        while(Robot.kElevator.doneyet != true){
          Robot.kElevator.complexElevate(20, 1);
          }
          Robot.kElevator.armup();
      }
      Robot.kDrivetrain.justdrive(.5);
      Robot.kElevator.targetIntake(4, .6);

    //Robot.kElevator.intakeup();



    }
//high ball
    else if(Robot.m_oi.balllow.get() == false && Robot.m_oi.ballmid.get() == false && Robot.m_oi.ballhigh.get() == true && Robot.m_oi.hatchlow.get() == false && Robot.m_oi.hatchmid.get() == false && Robot.m_oi.hatchhigh.get() == false){
      Robot.kElevator.doneyet = false;
      if(Robot.kElevator.intUP == false){
      while(Robot.kElevator.doneyet != true){
        Robot.kElevator.calmerComplex(78, 1);
      }
      Robot.kElevator.armup();
    }
    else{
      Robot.kElevator.targetIntake(31, .4);
    //  Robot.kDrivetrain.justdrive(1.3);
    Robot.kElevator.doneyet = false;
      while(Robot.kElevator.doneyet != true){
        Robot.kElevator.complexElevate(78, 1);
        }
        Robot.kElevator.armup();
    }  
        Robot.kDrivetrain.justdrive(.5);
        Robot.kElevator.targetIntake(4, .6);

    //Robot.kElevator.intakeup();


    }
//low hatch
    else if(Robot.m_oi.balllow.get() == false && Robot.m_oi.ballmid.get() == false && Robot.m_oi.ballhigh.get() == false && Robot.m_oi.hatchlow.get() == true && Robot.m_oi.hatchmid.get() == false && Robot.m_oi.hatchhigh.get() == false){
      Robot.kElevator.doneyet = false;
      if(Robot.kElevator.intUP == false){
      while(Robot.kElevator.doneyet != true){
      Robot.kElevator.calmerComplex(30, 1);
      }

      Robot.kElevator.armdown();
      Robot.kDrivetrain.justdrive(1.4);
      Robot.kElevator.calmerComplex(0, .6);
    }
    else{
      Robot.kElevator.targetIntake(31, .4);
     // Robot.kDrivetrain.justdrive(1.3);
     Robot.kElevator.doneyet = false;
      while(Robot.kElevator.doneyet != true){
        Robot.kElevator.calmerComplex(30, 1);
        }
  
        Robot.kElevator.armdown();
        Robot.kDrivetrain.justdrive(1.4);
        Robot.kElevator.calmerComplex(0, .6);
    }
      


    }
//mid hatch
    else if(Robot.m_oi.balllow.get() == false && Robot.m_oi.ballmid.get() == false && Robot.m_oi.ballhigh.get() == false && Robot.m_oi.hatchlow.get() == false && Robot.m_oi.hatchmid.get() == true && Robot.m_oi.hatchhigh.get() == false){
      Robot.kElevator.doneyet = false;
      if(Robot.kElevator.intUP == false){
      while(Robot.kElevator.doneyet != true){
      Robot.kElevator.complexElevate(78, 1); //14 before
      }
    //  Robot.kElevator.armup();
      }
      else{
        Robot.kElevator.targetIntake(31, .4);
        //Robot.kDrivetrain.justdrive(1.3);
        Robot.kElevator.doneyet = false;
        while(Robot.kElevator.doneyet != true){
          Robot.kElevator.complexElevate(78, 1);
          }
         // Robot.kElevator.armup();
      }
      Robot.kElevator.armdown();
      Robot.kDrivetrain.justdrive(.5);
      Robot.kElevator.targetIntake(4, .6);

    //Robot.kElevator.intakeup();
     //////
    // Robot.kDrivetrain.justdrive(.8);
     //Robot.kElevator.complexElevate(0, 1);
     /////


      /*
      if(Robot.kElevator.intUP == false){
      Robot.kElevator.complexElevate(78, 1);
      Robot.kElevator.armdown();
    }
      else{
        Robot.kElevator.targetIntake(31, .4);
        Robot.kDrivetrain.justdrive(1.3);
        Robot.kElevator.complexElevate(78, 1);
      Robot.kElevator.armdown();
      }
      Robot.kDrivetrain.justdrive(.5);

    //  //Robot.kElevator.intakeup();//
*/
    }
//high hatch
    else if(Robot.m_oi.balllow.get() == false && Robot.m_oi.ballmid.get() == false && Robot.m_oi.ballhigh.get() == false && Robot.m_oi.hatchlow.get() == false && Robot.m_oi.hatchmid.get() == false && Robot.m_oi.hatchhigh.get() == true){

     // Robot.kElevator.complexElevate(38, 1);
      //Robot.kElevator.armup();
      if(Robot.kElevator.intUP == false){
        Robot.kElevator.complexElevate(38, 1);
        Robot.kElevator.armup();
      }
        else{
          Robot.kElevator.targetIntake(31, .4);
          Robot.kDrivetrain.justdrive(1.3);
          Robot.kElevator.complexElevate(38, 1);
        Robot.kElevator.armup();
        }
        Robot.kDrivetrain.justdrive(1.5);
        Robot.kElevator.targetIntake(4, .6);

        //Robot.kElevator.intakeup();

    }
    else{
      Robot.kElevator.Up(0);
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
    Robot.kElevator.Up(0);

  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    end();
  }
}
