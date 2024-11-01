/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

/**
 * The RobotMap is a mapping from the ports sensors and actuators are wired into
 * to a variable name. This provides flexibility changing wiring, makes checking
 * the wiring easier and significantly reduces the number of magic numbers
 * floating around.
 */
public class RobotMap {
  public static WPI_TalonSRX leftdrive1 = new WPI_TalonSRX(3);
  public static WPI_TalonSRX rightdrive1 = new WPI_TalonSRX(1);
  public static CANSparkMax setl = new CANSparkMax(5, MotorType.kBrushless);
  public static CANSparkMax setr = new CANSparkMax(6, MotorType.kBrushless);
  public static void init() {
    rightdrive1.setInverted(true);
    leftdrive1.setInverted(true);

  }
  
}
