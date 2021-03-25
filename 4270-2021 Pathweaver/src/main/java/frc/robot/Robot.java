package frc.robot;

//import edu.wpi.cscore.UsbCamera;
//import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.commands.Driving;
import frc.robot.commands.Fast;
import frc.robot.commands.Hanging;
import frc.robot.commands.Indexing;
import frc.robot.commands.Intaking;
import frc.robot.commands.Shooter;
import frc.robot.commands.Spinwheeling;
//import frc.robot.commands.Auto.Auto1;
//import frc.robot.commands.Auto.Auto2;

import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Hang;
import frc.robot.subsystems.Index;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shifter;
import frc.robot.subsystems.Spinwheel;
import frc.robot.subsystems.Turret;
//import frc.robot.commands.Auto.AutoTest2;
import frc.robot.commands.Auto.Auto2;
import frc.robot.commands.Auto.AutoTest1;
import frc.robot.commands.Auto.Auto3;
//import jdk.nashorn.internal.ir.ThrowNode;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  public static String trajectoryJSON;

  // Command autonomusCommand;
  private Command autonomousCommand;
  static SendableChooser<Command> autoChooser = new SendableChooser<>();
  static SendableChooser<Command> startpos = new SendableChooser<>();

  // private RobotContainer m_robotContainer;

  public static OI m_oi;
  RobotContainer container;
  public static Drivetrain kDrivetrain = new Drivetrain();
  public static Driving kDriving = new Driving();

  public static Index kIndex = new Index();
  public static Indexing kIndexing = new Indexing(kIndex);
  public static Turret kTurret = new Turret();
  public static Shooter kShooter = new Shooter(kTurret);
  public static Intake kIntake = new Intake();
  public static Intaking kIntaking = new Intaking(kIntake);
  public static Shifter kShifter = new Shifter();
  public static Fast kFast = new Fast(kShifter);
  public static Hang kHang = new Hang();
  public static Hanging kHanging = new Hanging(kHang);
  public static Spinwheel kSpinwheel = new Spinwheel();
  public static Spinwheeling kSpinwheeling = new Spinwheeling(kSpinwheel);

  // private UsbCamera cam;

  /**
   * This function is run when the robot is first started up and should be used
   * for any initialization code.
   */
  @Override
  public void robotInit() {
    m_oi = new OI();
    container = new RobotContainer();
    RobotMap.init();

    // TODO: uncomment cam when reinstalled
    // cam = CameraServer.getInstance().startAutomaticCapture(0);
    // cam.setFPS(15);
    // cam.setResolution(320, 240);

    // Autonomous mode selector 

    // chooser.setDefaultOption("Test", new Auto2(kDrivetrain));
    // chooser.setDefaultOption("Auto", new Auto1(kDrivetrain));
    // chooser.setDefaultOption("Test", new AutoTest2());
    // chooser.setDefaultOption("Auto", new AutoTest1());
    // SmartDashboard.putData("Auto mode", chooser);
    autoChooser.setDefaultOption("Clear system", new AutoTest1());
    autoChooser.addOption("Statinary Turet", new Auto2());
    autoChooser.addOption("Drive 1m foward", new Auto3());
    SmartDashboard.putData("Autonomous routine", autoChooser);

    //Starting position selsctor
    // TODO: impliment the starting pos, olny the first path this wuld affect. 
    // Replace null tihe the starting pos name and in all the atounomouses files
    // have them get the selected item and changeg there first path acordingly.
    // may not need the red or blue sides as the field is identical on both sides.
    startpos.setDefaultOption("None",null);
    startpos.addOption("Blue1", null);
    startpos.addOption("Blue2", null);
    startpos.addOption("Blue3", null);
    startpos.addOption("Red1", null);
    startpos.addOption("Red2", null);
    startpos.addOption("Red3", null);
    SmartDashboard.putData("Starting position", startpos);




    // chooser.setDefaultOption("Test", 1);
    // chooser.addOption("Auto", 2);
    // SmartDashboard.putData("Auto mode", chooser);
  }

  /**
   * This function is called every robot packet, no matter the mode. Use this for
   * items like diagnostics that you want ran during disabled, autonomous,
   * teleoperated and test.
   *
   * <p>
   * This runs after the mode specific periodic functions, but before LiveWindow
   * and SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    /*
     * Runs the Scheduler. This is responsible for polling buttons, adding
     * newly-scheduled commands, running already-scheduled commands, removing
     * finished or interrupted commands, and running subsystem periodic() methods.
     * This must be called from the robot's periodic block in order for anything in
     * the Command-based framework to work.
     */

    CommandScheduler.getInstance().run();
    // cam.setFPS(15);
    // cam.setResolution(320, 240);

    // kShooter.schedule();

  }

  /**
   * This function is called once each time the robot enters Disabled mode.
   */
  @Override
  public void disabledInit() {
    // container.reset();
    kTurret.togglebtn = false;
    kTurret.unblindMe();
    // chooser.setDefaultOption("Test", AutoTest2);
    // chooser.addOption("Auto", AutoTest1);

  }

  @Override
  public void disabledPeriodic() {
    kTurret.unblindMe();
    Scheduler.getInstance().run();
    // SmartDashboard.putString("Auto Command", chooser.getSelected().getName());

  }

  /**
   * This autonomous runs the autonomous command selected by your
   * {@link RobotContainer} class.
   */
  @Override
  public void autonomousInit() {
    // container.reset();
    // Auto2 driveAuto = new Auto2(kDrivetrain);
    // kShifter.fast();
    // Auto1 driveAuto = new Auto1(kDrivetrain);
    // kDrivetrain.lowGear();
    // kTurret.unblindMe();
    autonomousCommand = container.getSelectedAuto(); // autoChooser.getSelected();

    // autonomousCommand = chooser.getSelected();

    // schedule the autonomous command (example)
    if (autonomousCommand != null) {
      autonomousCommand.schedule();
    }
  }

  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic() {
    // Scheduler.getInstance().run();

  }

  @Override
  public void teleopInit() {
    /*
     * This makes sure that the autonomous stops running when teleop starts
     * running. If you want the autonomous to continue until interrupted by
     * another command, remove this line or comment it out.
     */
    if (autonomousCommand != null) {
      autonomousCommand.cancel();
    }
  }

  /**
   * This function is called periodically during operator control.
   */
  @Override
  public void teleopPeriodic() {
    // container.reset();
    kDriving.schedule();
    kShooter.schedule();

    kIntaking.schedule();
    kIndexing.schedule();
    kFast.schedule();
    kHanging.schedule();
    kSpinwheeling.schedule();
    // robot.commands.Shooter();

  }

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic() {
  }
}
