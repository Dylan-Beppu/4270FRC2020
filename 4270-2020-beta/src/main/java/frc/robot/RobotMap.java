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
//import com.ctre.phoenix.*;
//import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.kauailabs.navx.frc.AHRS;
import com.kauailabs.navx.*;
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
  public static WPI_TalonSRX rightdrive1 = new WPI_TalonSRX(1);
  public static WPI_TalonSRX rightdrive2 = new WPI_TalonSRX(2);

  
  //Left drive
  public static WPI_TalonSRX leftdrive1 = new WPI_TalonSRX(3);
  public static WPI_TalonSRX leftdrive2 = new WPI_TalonSRX(4);

  //gryo
  public static AHRS gyro = new AHRS(SPI.Port.kMXP);

  public static CANSparkMax wristmotor = new CANSparkMax(10, MotorType.kBrushless);

  public static CANSparkMax wristmotor2 = new CANSparkMax(11, MotorType.kBrushless/**/);

  public static CANSparkMax visionm = new CANSparkMax(23, MotorType.kBrushless);
  public static CANSparkMax intake = new CANSparkMax(9, MotorType.kBrushless);

  public static CANSparkMax side = new CANSparkMax(18, MotorType.kBrushless);

  public static void init(){
    rightdrive1.setInverted(false);
    leftdrive1.setInverted(false);
    rightdrive2.setInverted(false);
    leftdrive2.setInverted(false);
  }
}
