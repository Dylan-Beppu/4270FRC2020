/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

//import edu.wpi.first.wpilibj.Compressor;

/*import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Ultrasonic;
import edu.wpi.first.wpilibj.Spark;/*/
import com.revrobotics.*;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
//import com.ctre.phoenix.*;
//import com.ctre.phoenix.motorcontrol.can.TalonSRX;
//import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.kauailabs.navx.frc.AHRS;
import com.kauailabs.navx.*;

import edu.wpi.first.wpilibj.AnalogEncoder;
import edu.wpi.first.wpilibj.CAN;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.PWM;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.SerialPort.Port;
import edu.wpi.first.wpilibj.util.Color;

import com.revrobotics.ColorSensorV3;
import com.revrobotics.ColorMatchResult;
import com.revrobotics.ColorMatch;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;

/**
 * The RobotMap is a mapping from the ports sensors and actuators are wired into
 * to a variable name. This provides flexibility changing wiring, makes checking
 * the wiring easier and significantly reduces the number of magic numbers
 * floating around.
 */
public class RobotMap {
  //public static Compressor aircomp = new Compressor(0);

 //Right drive
  public static WPI_TalonFX rightdrive1 = new WPI_TalonFX(1);
  public static WPI_TalonFX rightdrive2 = new WPI_TalonFX(2);

  
  //Left drive
  public static WPI_TalonFX leftdrive1 = new WPI_TalonFX(3);
  public static WPI_TalonFX leftdrive2 = new WPI_TalonFX(4);

  //gryo
  public static AHRS gyro = new AHRS(Port.kUSB);

  //NEOs
  public static CANSparkMax LeftIntake = new CANSparkMax(5, MotorType.kBrushless);
  public static CANSparkMax RightIntake = new CANSparkMax(6, MotorType.kBrushless);
  public static CANSparkMax CenterIntake = new CANSparkMax(13, MotorType.kBrushless);
  public static CANSparkMax IndexBottom = new CANSparkMax(7, MotorType.kBrushless);
  public static CANSparkMax IndexTop = new CANSparkMax(8, MotorType.kBrushless);
  public static CANSparkMax FlyboiL = new CANSparkMax(9, MotorType.kBrushless);
  public static CANSparkMax FlyboiR = new CANSparkMax(10, MotorType.kBrushless);
  public static CANSparkMax Rotateboi = new CANSparkMax(11, MotorType.kBrushless);
  public static CANSparkMax spin1 = new CANSparkMax(12, MotorType.kBrushless);
  
  //color sensor
  public static final I2C.Port i2cPort = I2C.Port.kOnboard;
  public static final ColorSensorV3 colorSensor = new ColorSensorV3(i2cPort);
  public static final ColorMatch colorMatcher = new ColorMatch();

  public static final Color kBlueTarget = ColorMatch.makeColor(0.143, 0.427, 0.429);
  public static final Color kGreenTarget = ColorMatch.makeColor(0.197, 0.561, 0.240);
  public static final Color kRedTarget = ColorMatch.makeColor(0.561, 0.232, 0.114);
  public static final Color kYellowTarget = ColorMatch.makeColor(0.361, 0.524, 0.113);


  //pneumatics stuff
  public static Compressor aircomp = new Compressor(0);

  //note change chanles
  public static DoubleSolenoid shifter = new DoubleSolenoid(0, 0, 1);
  public static DoubleSolenoid arm = new DoubleSolenoid(0, 2, 3);
  
  
  public static void init(){
    rightdrive1.setInverted(false);
    leftdrive1.setInverted(false);
    rightdrive2.setInverted(false);
    leftdrive2.setInverted(false);
  }
}
