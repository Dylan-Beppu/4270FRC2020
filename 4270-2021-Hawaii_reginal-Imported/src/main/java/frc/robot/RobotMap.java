package frc.robot;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
//import com.ctre.phoenix.*;
//import com.ctre.phoenix.motorcontrol.can.TalonSRX;
//import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.kauailabs.navx.frc.AHRS;

//import edu.wpi.first.wpilibj.Compressor;

/*import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Ultrasonic;
import edu.wpi.first.wpilibj.Spark;/*/
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
//import com.revrobotics.jni.CANSparkMaxJNI;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PWMSparkMax;
import edu.wpi.first.wpilibj.SerialPort.Port;

/**
 * The RobotMap is a mapping from the ports sensors and actuators are wired into
 * to a variable name. This provides flexibility changing wiring, makes checking
 * the wiring easier and significantly reduces the number of magic numbers
 * floating around.
 */
public class RobotMap {
 //Right drive
  public static WPI_TalonFX rightdrive1 = new WPI_TalonFX(3);
  public static WPI_TalonFX rightdrive2 = new WPI_TalonFX(4);

  //Left drive
  public static WPI_TalonFX leftdrive1 = new WPI_TalonFX(1);
  public static WPI_TalonFX leftdrive2 = new WPI_TalonFX(2);

  //gryo
 public static AHRS gyro = new AHRS(Port.kUSB);

  //Index
  public static CANSparkMax LeftIntake = new CANSparkMax(5, MotorType.kBrushless);
  public static CANSparkMax CenterIntake = new CANSparkMax(6, MotorType.kBrushless);
  public static CANSparkMax IndexBottom = new CANSparkMax(7, MotorType.kBrushless);

  //Flywhel
  public static CANSparkMax Rotateboi = new CANSparkMax(8, MotorType.kBrushless);
  public static CANSparkMax FlyboiL = new CANSparkMax(9, MotorType.kBrushless);
  public static CANSparkMax Topin = new CANSparkMax(10, MotorType.kBrushless);
  public static CANSparkMax FlyboiR = new CANSparkMax(11, MotorType.kBrushless);

  //other
  public static CANSparkMax Intake = new CANSparkMax(12, MotorType.kBrushless);
  public static CANSparkMax Endgame = new CANSparkMax(13, MotorType.kBrushless);
  
  public static PWMSparkMax R2 = new PWMSparkMax(0);
  
  //color sensor
  //public static final I2C.Port i2cPort = I2C.Port.kOnboard;
  //public static final ColorSensorV3 colorSensor = new ColorSensorV3(i2cPort);
  //public static final ColorMatch colorMatcher = new ColorMatch();

  //color values
  //public static final Color kBlueTarget = ColorMatch.makeColor(0.143, 0.427, 0.429);
  //public static final Color kGreenTarget = ColorMatch.makeColor(0.197, 0.561, 0.240);
  //public static final Color kRedTarget = ColorMatch.makeColor(0.561, 0.232, 0.114);
  //public static final Color kYellowTarget = ColorMatch.makeColor(0.361, 0.524, 0.113);

  //beam brake
  public static DigitalInput BeamBrakeTop = new DigitalInput(8);
  public static DigitalInput BeamBrakeBotom = new DigitalInput(9);

  public static UsbCamera Cam = new UsbCamera("camm", 1);
  //pneumatics stuff
  public static Compressor aircomp = new Compressor(0);

  //note change chanles
  //out|highgear 0 // in|lowgar 1
  public static DoubleSolenoid shifter = new DoubleSolenoid(0, 0, 1);
  //in|up 2 //out|down 3
  public static DoubleSolenoid arm = new DoubleSolenoid(0, 2, 3);
  //in|down 4 //out|up 5
  public static DoubleSolenoid hood = new DoubleSolenoid(0 , 6, 7);
  
  //flip here if dont work
  public static DoubleSolenoid releasH = new DoubleSolenoid(0 , 4, 5);
  
  public static void init(){
    rightdrive1.configFactoryDefault();
    rightdrive2.configFactoryDefault();
    leftdrive1.configFactoryDefault();
    leftdrive2.configFactoryDefault();
    rightdrive1.setInverted(true);
    rightdrive2.setInverted(true);
    leftdrive1.setInverted(false);
    leftdrive2.setInverted(false);
    rightdrive2.setNeutralMode(NeutralMode.Brake);
    rightdrive1.setNeutralMode(NeutralMode.Coast );
    leftdrive1.setNeutralMode(NeutralMode.Brake);
    leftdrive2.setNeutralMode(NeutralMode.Coast);
    Topin.setIdleMode(IdleMode.kCoast);
    RobotMap.FlyboiL.setIdleMode(IdleMode.kCoast);
    RobotMap.FlyboiR.setIdleMode(IdleMode.kCoast);
    Endgame.restoreFactoryDefaults();
    Endgame.setIdleMode(IdleMode.kBrake);
  }
}
