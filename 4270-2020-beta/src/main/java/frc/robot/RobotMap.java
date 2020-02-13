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
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.PWM;
import edu.wpi.first.wpilibj.SPI;


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
  public static AHRS gyro = new AHRS(SPI.Port.kMXP);

  public static CANSparkMax LeftIntake = new CANSparkMax(5, MotorType.kBrushless);
  public static CANSparkMax RightIntake = new CANSparkMax(6, MotorType.kBrushless);
  public static CANSparkMax IndexBottom = new CANSparkMax(7, MotorType.kBrushless);
  public static CANSparkMax IndexTop = new CANSparkMax(8, MotorType.kBrushless);
  public static CANSparkMax FlyboiL = new CANSparkMax(9, MotorType.kBrushless);
  public static CANSparkMax FlyboiR = new CANSparkMax(10, MotorType.kBrushless);
  public static CANSparkMax Rotateboi = new CANSparkMax(11, MotorType.kBrushless);
  public static CANSparkMax spin1 = new CANSparkMax(12, MotorType.kBrushless);


  public static CANSparkMax side = new CANSparkMax(18, MotorType.kBrushless);
  
  //for versa planitary encoder
  public static Encoder TurretEnc = new Encoder(0,1);

  public static void init(){
    rightdrive1.setInverted(false);
    leftdrive1.setInverted(false);
    rightdrive2.setInverted(false);
    leftdrive2.setInverted(false);
  }
}
