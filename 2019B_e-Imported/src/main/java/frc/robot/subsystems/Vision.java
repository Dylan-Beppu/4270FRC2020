/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTableValue;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.buttons.Button;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.robot.commands.Visiontest;
import frc.robot.driver.Limelight;
import frc.robot.OI;


/**
 * Add your docs here.
 */
public class Vision extends Subsystem {
  NetworkTableEntry tx;
  NetworkTableEntry ty;

  
  // Put methods for controlling this subsystem
  // here. Call these from Commands.
  // Example: NetworkTableInstance.getDefault().getTable("limelight").getEntry("<variablename>").getDouble(0);

  // limelight values
  // tv	Whether the limelight has any valid targets (0 or 1)
  // tx	Horizontal Offset From Crosshair To Target (LL1: -27 degrees to 27 degrees | LL2: -29.8 to 29.8 degrees) !!!
  // ty	Vertical Offset From Crosshair To Target (LL1: -20.5 degrees to 20.5 degrees | LL2: -24.85 to 24.85 degrees)
  // ta	Target Area (0% of image to 100% of image)
  // ts	Skew or rotation (-90 degrees to 0 degrees)
  // tl	The pipeline’s latency contribution (ms) Add at least 11ms for image capture latency.

  //talon can id 13
  private double Xangle;

  private final TalonSRX twist = RobotMap.VisionTurn;
  //private  Xangle = NetworkTableInstance.getDefault().getTable("table").getEntry("<ty>").getValue();
  //private frc.robot.driver.Limelight.;

  @Override
  public void initDefaultCommand() {
   //setDefaultCommand(new mtwist());
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }
  public void track(){
       //Get the default instance of NetworkTables that was created automatically
       //when your program starts
       NetworkTableInstance inst = NetworkTableInstance.getDefault();

       //Get the table within that instance that contains the data. There can
       //be as many tables as you like and exist to make it easier to organize
       //your data. In this case, it's a table called datatable.
       NetworkTable table = inst.getTable("limelight");

       //Get the entries within that table that correspond to the X and Y values
       //for some operation in your program.
       tx = table.getEntry("tx");
       ty = table.getEntry("ty");

    if(tx.getValue().getDouble() >= 1 && Robot.m_oi.stick.getRawButtonPressed(3)){
      //twist.set();
      twist.set(ControlMode.PercentOutput, 0.05);
    }
    else if((tx.getValue().getDouble() >= 1 && Robot.m_oi.stick.getRawButtonPressed(3))){
      twist.set(ControlMode.PercentOutput, 0.05);
    }
    else{
      twist.set(ControlMode.PercentOutput, 0);
    }
  }
}
