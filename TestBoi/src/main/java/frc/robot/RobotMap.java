/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.SPI;

/*import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Ultrasonic;
import edu.wpi.first.wpilibj.Spark;/*/
import com.revrobotics.*;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
//import com.ctre.phoenix.*;
//import ;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.kauailabs.navx.frc.AHRS;
import com.kauailabs.navx.*;

/**
 * The RobotMap is a mapping from the ports sensors and actuators are wired into
 * to a variable name. This provides flexibility changing wiring, makes checking
 * the wiring easier and significantly reduces the number of magic numbers
 * floating around.
 */
public class RobotMap {
  //public static Compressor aircomp = new Compressor(0);

  //Right drive
  //public static TalonSRX rightdrive1 = new TalonSRX(1);
  //public static TalonSRX rightdrive2 = new TalonSRX(2);
  public static CANSparkMax rightdrive1 = new CANSparkMax(5,CANSparkMaxLowLevel.MotorType.kBrushless);
  
  //Left drive
  //public static TalonSRX leftdrive1 = new TalonSRX(3);
  //public static TalonSRX leftdrive2 = new TalonSRX(4);
  public static CANSparkMax leftdrive1 = new CANSparkMax(6,CANSparkMaxLowLevel.MotorType.kBrushless);
  //public static CANEncoder leftCanEncoder = new CANEncoder(leftdrive1);


  public static AHRS gyro = new AHRS(SPI.Port.kMXP);
  public static void init(){
    rightdrive1.setInverted(false);
    leftdrive1.setInverted(false);
  }
  // If you are using multiple modules, make sure to define both the port
  // number and the module. For example you with a rangefinder:
  // public static int rangefinderPort = 1;
  // public static int rangefinderModule = 1;
}
