/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.Compressor;
/*import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Ultrasonic;
import edu.wpi.first.wpilibj.Spark;/*/
import com.revrobotics.*;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
//import com.ctre.phoenix.*;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

/**
 * The RobotMap is a mapping from the ports sensors and actuators are wired into
 * to a variable name. This provides flexibility changing wiring, makes checking
 * the wiring easier and significantly reduces the number of magic numbers
 * floating around.
 */
public class RobotMap {
  //public static Compressor aircomp = new Compressor(0);

  //Right drive
  public static TalonSRX rightdrive1 = new TalonSRX(1);
  public static TalonSRX rightdrive2 = new TalonSRX(2);

  //Left drive
  public static TalonSRX leftdrive1 = new TalonSRX(3);
  public static TalonSRX leftdrive2 = new TalonSRX(4);

  //Flywheel
  public static CANSparkMax Flyboy = new CANSparkMax(5, MotorType.kBrushless);
  public static CANSparkMax Flap = new CANSparkMax(6, MotorType.kBrushless);

  //spinner
  public static CANSparkMax R2D2 = new CANSparkMax(7, MotorType.kBrushless);

  //compressor
  public static Compressor CompresBoy;


  // For example to map the left and right motors, you could define the
  // following variables to use with your drivetrain subsystem.
  // public static int leftMotor = 1;
  // public static int rightMotor = 2;

  // If you are using multiple modules, make sure to define both the port
  // number and the module. For example you with a rangefinder:
  // public static int rangefinderPort = 1;
  // public static int rangefinderModule = 1;
}
