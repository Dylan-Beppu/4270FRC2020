/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Ultrasonic;

//import edu.wpi.first.wpilibj.Spark;
import com.revrobotics.*;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
//import com.ctre.phoenix.*;

/**
 * The RobotMap is a mapping from the ports sensors and actuators are wired into
 * to a variable name. This provides flexibility changing wiring, makes checking
 * the wiring easier and significantly reduces the number of magic numbers
 * floating around.
 */
public class RobotMap {
  

//public static CANSparkMax exampleSpark = new CANSparkMax(69, MotorType.kBrushless);

public static Compressor aircomp = new Compressor(0);

public static CANSparkMax rightmasterdrive = new CANSparkMax(1, MotorType.kBrushless);
public static CANSparkMax rightdrive1 = new CANSparkMax(2, MotorType.kBrushless);
public static CANSparkMax rightdrive2 = new CANSparkMax(3, MotorType.kBrushless);

public static CANSparkMax leftmasterdrive = new CANSparkMax(4, MotorType.kBrushless);
public static CANSparkMax leftdrive1 = new CANSparkMax(5, MotorType.kBrushless);
public static CANSparkMax leftdrive2 = new CANSparkMax(6, MotorType.kBrushless);


public static CANSparkMax Melevator = new CANSparkMax(7, MotorType.kBrushless);
public static CANSparkMax Felevator = new CANSparkMax(8, MotorType.kBrushless);

public static CANSparkMax intake = new CANSparkMax(9, MotorType.kBrushless);

public static CANSparkMax side = new CANSparkMax(18, MotorType.kBrushless);

public static DoubleSolenoid shifter = new DoubleSolenoid(0, 0, 1);
public static DoubleSolenoid arm = new DoubleSolenoid(0, 2, 3);
public static DoubleSolenoid FrontClimb = new DoubleSolenoid(0, 4, 5);
public static DoubleSolenoid BackClimb = new DoubleSolenoid(0, 6, 7);

public static DoubleSolenoid Probe = new DoubleSolenoid(1, 0, 1);
public static DoubleSolenoid Grab = new DoubleSolenoid(1, 2, 3);
public static DoubleSolenoid SolenoidIntake = new DoubleSolenoid(1, 4, 5);

public static DigitalInput BottomLimit = new DigitalInput(0);
public static DigitalInput intakeLimit = new DigitalInput(3);

public static Ultrasonic ultra = new Ultrasonic(1, 2);

//public static TalonSRX wristmotor = new TalonSRX(1/**/);
public static CANSparkMax wristmotor = new CANSparkMax(10, MotorType.kBrushless);

public static CANSparkMax wristmotor2 = new CANSparkMax(11, MotorType.kBrushless/**/);



  // For example to map the left and right motors, you could define the
  // following variables to use with your drivetrain subsystem.
  // public static int leftMotor = 1;
  // public static int rightMotor = 2;
  public static void init() {
  //  rightdrive1.follow(rightmasterdrive);
    //rightdrive2.follow(rightmasterdrive);

  //  leftdrive1.follow(leftmasterdrive);
    //leftdrive2.follow(leftmasterdrive);
   // Felevator.follow(Melevator);
    
    //leftmasterdrive.setInverted(true);
    leftdrive1.setInverted(true);
    //leftdrive2.setInverted(true);

    rightmasterdrive.setInverted(false);
    rightdrive1.setInverted(true);
    rightdrive2.setInverted(true);
    
    wristmotor.setInverted(true);
    //wristmotor2.follow(wristmotor);
    wristmotor2.setInverted(true);

    //exampleSpark.setInverted(true);

  }
  // If you are using multiple modules, make sure to define both the port
  // number and the module. For example you with a rangefinder:
  // public static int rangefinderPort = 1;
  // public static int rangefinderModule = 1;
}
