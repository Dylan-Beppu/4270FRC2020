/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.commands.ExampleCommand;
//import frc.robot.commands.*;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.testsub;
import frc.robot.subsystems.*;
//import frc.robot.driver.Limelight;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  public static ExampleSubsystem m_subsystem = new ExampleSubsystem();
  public static testsub kTestsub = new testsub();
  public static compress kcompress = new compress();
  public static drivetrain kDrivetrain = new drivetrain();
  public static elevator kElevator = new elevator();
  public static intake kIntake = new intake();
  public static fourbar kFourbar = new fourbar();
  public static shifter kShifter = new shifter();
  public static probe kProbe = new probe();
  public static pinch kpinch = new pinch();
  public static Ultra kUltra = new Ultra();
  public static roller kroller = new roller();
  public static vision kvision = new vision();
  public static OI m_oi;

  Command m_autonomousCommand;
  SendableChooser<Command> m_chooser = new SendableChooser<>();
  private UsbCamera camera1;
  private UsbCamera camera0;

  /**
   * This function is run when the robot is first started up and should be
   * used for any initialization code.
   */
  @Override
  public void robotInit() {
    camera1 = CameraServer.getInstance().startAutomaticCapture(1);
    camera1.setFPS(15);
    camera1.setResolution(320, 240);
    camera0 = CameraServer.getInstance().startAutomaticCapture(0);
    camera0.setFPS(15);
    camera0.setResolution(320, 240);

    //CameraServer.getInstance().startAutomaticCapture(1);

   // CameraServer.getInstance().addServer("Vision");
    m_oi = new OI();
    m_chooser.setDefaultOption("Default Auto", new ExampleCommand());
    // chooser.addOption("My Auto", new MyAutoCommand());
    SmartDashboard.putData("Auto mode", m_chooser);
    RobotMap.init();
    kcompress.start();

    RobotMap.Melevator.getEncoder().setPosition(0);
    RobotMap.wristmotor.getEncoder().setPosition(0);
    RobotMap.wristmotor2.getEncoder().setPosition(0);
    RobotMap.ultra.setAutomaticMode(true);

    kElevator.intUP = true;
    kShifter.slow();
    kShifter.isfast = true;
    //kElevator.armdown();

    kvision.togglebtn = false;

//
   /// RobotMap.ultra.pidGet();
  //  double Robot
    //
  }

  /**
   * This function is called every robot packet, no matter the mode. Use
   * this for items like diagnostics that you want ran during disabled,
   * autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before
   * LiveWindow and SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    camera1.setFPS(15);
    camera1.setResolution(320, 240);
    camera0.setFPS(15);
    camera0.setResolution(320, 240);

    /*RobotMap.ultra.pidGet();
    double rawrange = RobotMap.ultra.getAverageVoltage() * 36.6;
    double range = rawrange + 12;
*/
    SmartDashboard.putNumber("Elevator position", RobotMap.Melevator.getEncoder().getPosition());
    SmartDashboard.putBoolean("Bottom Limit", RobotMap.BottomLimit.get());
    SmartDashboard.putNumber("Felevator position", RobotMap.Felevator.getEncoder().getPosition());
    SmartDashboard.putBoolean("Pinch", kpinch.lionholler);
   
    SmartDashboard.putBoolean("Ultra Enabled", RobotMap.ultra.isEnabled());
    SmartDashboard.putBoolean("Ultra valid", RobotMap.ultra.isRangeValid());
    SmartDashboard.putNumber("Ultra rangeMM", RobotMap.ultra.getRangeMM());
    SmartDashboard.putNumber("Ultra Range Inch", RobotMap.ultra.getRangeInches());
    SmartDashboard.putBoolean("isFast", kShifter.isfast);
    SmartDashboard.putNumber("Intake Wrist Pos", RobotMap.wristmotor.getEncoder().getPosition());
    SmartDashboard.putBoolean("Intake Button", RobotMap.intakeLimit.get());
   // SmartDashboard.putNumber("Intake Wrist2 Pos", RobotMap.wristmotor2.getSelectedSensorPosition());
    SmartDashboard.putBoolean("Intake UP?", kElevator.intUP);
   /*
    SmartDashboard.putNumber("UltraAVGVoltage", RobotMap.ultra.getAverageVoltage());
    SmartDashboard.putNumber("Range", range);
    SmartDashboard.putNumber("Avg Value", RobotMap.ultra.getAverageValue());
  */
  //SmartDashboard.putNumber("vision angle", SmartDashboard);
  }

  /**
   * This function is called once each time the robot enters Disabled mode.
   * You can use it to reset any subsystem information you want to clear when
   * the robot is disabled.
   */
  @Override
  public void disabledInit() {
  }

  @Override
  public void disabledPeriodic() {
    Scheduler.getInstance().run();
  }

  /**
   * This autonomous (along with the chooser code above) shows how to select
   * between different autonomous modes using the dashboard. The sendable
   * chooser code works with the Java SmartDashboard. If you prefer the
   * LabVIEW Dashboard, remove all of the chooser code and uncomment the
   * getString code to get the auto name from the text box below the Gyro
   *
   * <p>You can add additional auto modes by adding additional commands to the
   * chooser code above (like the commented example) or additional comparisons
   * to the switch structure below with additional strings & commands.
   */
  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_chooser.getSelected();

    /*
     * String autoSelected = SmartDashboard.getString("Auto Selector",
     * "Default"); switch(autoSelected) { case "My Auto": autonomousCommand
     * = new MyAutoCommand(); break; case "Default Auto": default:
     * autonomousCommand = new ExampleCommand(); break; }
     */

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.start();
    }
  }

  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic() {
    Scheduler.getInstance().run();
  }

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
    kElevator.armdown();
  }

  /**
   * This function is called periodically during operator control.
   */
  @Override
  public void teleopPeriodic() {
    Scheduler.getInstance().run();
  }

  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic() {
  }
}
