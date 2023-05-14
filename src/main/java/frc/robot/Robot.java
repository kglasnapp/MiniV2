/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;


import static frc.robot.utilities.Util.logf;
import static frc.robot.utilities.Util.round2;
import static frc.robot.utilities.Util.splashScreen;

import edu.wpi.first.hal.can.CANStatus;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;

/* Imports from template
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.CommandBase;
//import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.commands.ExampleCommand;
import frc.robot.subsystems.ExampleSubsystem; */

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.commands.groups.Autonomous;
import frc.robot.commands.groups.Autonomous.AutonomousMode;
import frc.robot.subsystems.ChameleonVision;
import frc.robot.subsystems.DistanceSensors;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Joysticks;
import frc.robot.subsystems.Leds;
import frc.robot.subsystems.Lidar;
import frc.robot.subsystems.LimeLightVision;
import frc.robot.subsystems.NeoPixelControl;
import frc.robot.subsystems.NeoPixelControl.LedAssignment;
import frc.robot.subsystems.NeoPixelControl.LedType;
import frc.robot.subsystems.OpensightVision;
import frc.robot.subsystems.Pigeon;
import frc.robot.subsystems.TurretShooter;
import frc.robot.subsystems.YawProvider;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {

  /*
  public static ExampleSubsystem m_subsystem = new ExampleSubsystem();
  public static OI m_oi;

  Command m_autonomousCommand;
  SendableChooser<Command> m_chooser = new SendableChooser<>(); */

  public static long count = 0;
  private String version = "0.7";
  //public static ScheduleCommand scheduler = ScheduleCommand.getInstance();
  public static Drive drive;
  public static Joysticks joysticks;
  public static Config config;
  public static YawProvider yawSensor = new YawProvider();
  public static double yaw; // Robots yaw as determined by the active Yaw sensor
  public static boolean logging = false;
  public static double frontDist = 0;
  public static double rearDist = 0;
  public static Leds leds;
  public static LimeLightVision limeLightVision;
  public static ChameleonVision chameleonVision;
  public static OpensightVision opensight;
  public static NeoPixelControl neoPixelControl;
  public static DistanceSensors distanceSensors;
  public static OI oi;
  public static double batteryVoltage = 12.5;
  public static Lidar lidar;
  public static Pigeon pigeon;
  public static TurretShooter turretShooter;
  private CommandBase autonomousCommand;
  private SendableChooser<AutonomousMode> positionChooser; // = new SendableChooser();

  // https://docs.wpilib.org/en/latest/docs/software/wpilib-overview/creating-robot-program.html#timedrobot
  public Robot() {
    //super(0.05); // Periodic methods will now be called every 20 ms.
    config = new Config();
    splashScreen(version);
    neoPixelControl = new NeoPixelControl();
    joysticks = new Joysticks();
    drive = new Drive(logging);
    leds = new Leds();
    turretShooter = new TurretShooter();
    //compressor = new CompressorController();

    if (config.limeLight)
      limeLightVision = new LimeLightVision();
    if (config.opensightVision)
      opensight = new OpensightVision();
    if (config.chameleonVision)
      chameleonVision = new ChameleonVision();
    distanceSensors = new DistanceSensors();
    if (config.lidar)
      lidar = new Lidar();
    if (config.pigeon)
      pigeon = new Pigeon();
    oi = new OI();
    yawSensor.zeroYaw();

  //Autonomous Modes
    positionChooser = new SendableChooser<AutonomousMode>();
    positionChooser.addOption("Test", AutonomousMode.Test);
    SmartDashboard.putData("Autonomous Position", positionChooser);
  }

  /**
   * This function is run when the robot is first started up and should be
   * used for any initialization code.
   */
  @Override
  public void robotInit() {
    /*
    m_oi = new OI();
    m_chooser.setDefaultOption("Default Auto", new ExampleCommand());
    // chooser.addOption("My Auto", new MyAutoCommand());
    SmartDashboard.putData("Auto mode", m_chooser); */


    //logf("***** Autonomous mode: %s *****\n", positionChooser.toString());
    //autonomousCommand = new Autonomous(positionChooser.getSelected());
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
    CommandScheduler.getInstance().run();
    count++;
  }


  /**
   * This function is called once each time the robot enters Disabled mode.
   * You can use it to reset any subsystem information you want to clear when
   * the robot is disabled.
   */
  @Override
  public void disabledInit() {
    neoPixelControl.setLed(LedAssignment.Enabled, LedType.YELLOW);
  }

  @Override
  public void disabledPeriodic() {
    //Scheduler.getInstance().run();
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
    //m_autonomousCommand = m_chooser.getSelected();

    /*
     * String autoSelected = SmartDashboard.getString("Auto Selector",
     * "Default"); switch(autoSelected) { case "My Auto": autonomousCommand
     * = new MyAutoCommand(); break; case "Default Auto": default:
     * autonomousCommand = new ExampleCommand(); break; }
     */

    // schedule the autonomous command (example)
    //if (m_autonomousCommand != null) {
      //m_autonomousCommand.start();
    //}

    // schedule the autonomous command (example)
    logf("***** Init Autonomous mode: %s *****\n", positionChooser.getSelected());
    autonomousCommand = new Autonomous(positionChooser.getSelected());
    if (autonomousCommand != null)
      autonomousCommand.schedule();;
  }

  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic() {
  }

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. 
    if (autonomousCommand != null) {
      autonomousCommand.cancel();
    }
    neoPixelControl.setLed(LedAssignment.Enabled, LedType.GREEN);
  }

  /**
   * This function is called periodically during operator control.
   */
  @Override
  public void teleopPeriodic() {
    long startTime = RobotController.getFPGATime();
    drive.periodic(); // todo is this needed if you override periodic in the drive.java
    turretShooter.periodic();
    health(startTime, false);
    yaw = yawSensor != null ? yawSensor.yaw() : 0; // Read the yaw sensor
    if (count % 10 == 0) { // Update the Dashboard with the YAW every 200 ms.
      SmartDashboard.putNumber("Yaw", round2(yaw)); // Put the yaw to Smart Dashboard
      if (oi.toggleDetailedLogging()) {
        logging = !logging;
        logf("Logging is:%s\n", logging ? "On" : "Off");
      }
    }
    if (count % (50 * 60) == 0) {
      batteryVoltage = RobotController.getBatteryVoltage();
      // % Utilization, bussOffCount, txCount, rxErrorCount, txErrorCount
      CANStatus status = RobotController.getCANStatus();
      String gData = DriverStation.getGameSpecificMessage();
      logf("CAN Status Utilization:%.2f bussOff:%d txCount:%d txErr:%d rxErr:%d\n", status.percentBusUtilization,
          status.busOffCount, status.txFullCount, status.transmitErrorCount, status.receiveErrorCount);
      logf("Yaw:%.2f Ultra Back:%.1f Front:%.1f Encoders Right:%d Left:%d Battery:%.2f Count:%d GameData:%s\n", yaw,
          rearDist, frontDist, drive.rightEncoder(), drive.leftEncoder(), batteryVoltage, count, gData);
    }
  }

  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic() {
  }

  static long lastFree = 0; // variable to hold the free memory last time health check was done
  static long lastCollection = 0; // Time when garbage collections (gc) was done

  void health(long startTime, boolean logGC) {
    if (count % 100 == 0) { // Do health check every 2 seconds
      long free = Runtime.getRuntime().freeMemory();
      // Will playing with this, reduce the impact of gc
      // if (count % 500 == 0) {
      // logf("Do a forced gc count:%d mem:%d", count, free);
      // Runtime.getRuntime().gc();
      // }
      if (logGC) {
        if (free > lastFree) {
          // Garbage collection has happened -- collect the data
          logf("Garbage Collection count:%d freed:%d elapsed:%.1f\n", count, free - lastFree,
              (startTime - lastCollection) / 1000000.0);
          lastCollection = startTime;
        }
        lastFree = free;
      }
      // Show free memory on the dashboard
      SmartDashboard.putNumber("Free Mem", free);
    }
  }
}
