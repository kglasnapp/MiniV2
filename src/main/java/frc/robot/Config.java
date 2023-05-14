package frc.robot;

import static frc.robot.utilities.Util.*;
import java.io.*;

import frc.robot.subsystems.PID;

public class Config {

    public enum RobotType {
        RevMini, SRXMini, RevChassis, Competition, SuitCase, SRXMiniV2, Shooter
    };

    public static RobotType robotType = RobotType.Shooter;

    // Pneumatic Control Modules Parameters
    public boolean pcmID0 = false;
    public boolean pcmID1 = false;

    // Vision Parameters
    public boolean limeLight = false;
    public boolean opensightVision = false;
    public boolean ledRing = false;
    public boolean chameleonVision = false;
    public boolean limeLightTurret = true;

    public enum DriveType {
        None, Tank, SingleJoy, OperatorTank, OperatorSingleJoy, OperatorMecanum
    };

    // Drive Parameters todo correct drive params
    public int driveRight = 2;
    public int driveLeft = 3;
    public int driveRightFollow = 4;
    public int driveLeftFollow = 5;
    public DriveType driveType = DriveType.OperatorTank;
    public boolean dualDriveMotors = false;
    public double driveTicksPerInch = 116; // 116 is for Mini's with SRX and 6" wheels
    public boolean defaultBrakeMode = true;
    public double wheelBase = 15.5; // Wheel base for mini
    public double elevatorTicksPerInch = 20 * 200; // Need to get actual measurement after testing
    public double wheelDiameter = 6.0; // Wheel diameter for MINI
    public double driveTicksPerRevolution = 2000; // Value for mini
    public double throttleRate = 1.5; // the exponetial for the throttle power curve
    public PID positionPID;
    public PID velocityPID;
    public double actualTurnAngle45 = 39;

    // Elevator Parameters
    public int elevatorID = -1;
    public double defaultElevatorMinMax = 1.0;
    public boolean elevatorControlViaTriggers = true;
    public double elevatorPresets[] = { 0, 2, 6, 10, 12, 15 }; // Presets for the elevator in inches

    // Miscellaneous Parameter
    public int kTimeoutMs = 30; // default timeout used for messages to the SRX
    public boolean enableCompressor = false;
    public boolean joysticksEnabled = false;
    public boolean operatorPadEnabled = true;
    public boolean neoPixelsActive = true;
    public double deadZone = 0.085;

    // Shooter parameters
    public boolean shooterVelocityPID = true;
    public int bottomShooterID = -1;
    public int topShooterID = -1;
    public int revRPM = 5676;
    public int turretID = 18;
 
    // Ball Conveyor parameters
    public int ballConveyorID = -18;
    public int intakeConveryorID = -22;
    public int ballInConveyorStartPosition = 5;
    public int ballInShooterPosition = 6;
    public int ticksPerBall = 0;
    public double ballConveyorSpeed = -0.7;

    // Winch -- Lifter for the Robot
    public int winchID = -11;
    public double winchUpSpeed = 0.8;

    // Telescope -- Hook Deployment for Robot
    public int telescopeID = -14;
    public double telescopeUpSpeed = .7;
    public double telescopeCurrentLimit = 0.0;

    // Orienting Shooter
    public int shooterElevateID = -1;

    // Was actual:<a:-7.5,p:-750375.0> or actual:<a:-7.5,p:-750375.0> with 10000 *
    // 10
    public double ticksPerDegreeEl = 16675;

    // Exit velocity of ball measured from testing meters per second
    // power bottom 1, top 0.6
    public double shooterExitVelocity = 2.2448;

    // Intake parameters
    public int intakeHorizontalID = -1;
    public int intakeVerticalID = -1;
    public double intakeSpeed = 0.25 * 1.2;

    // Misc parameters
    public boolean lidar = false;
    public boolean disableOperatorRightJoyDuringDriving = false;
    public boolean pigeon = false;

    // Testing functions
    public boolean testREVDriveonMini = false; // Set true if you want to simulate REV motor for drive tests

    // Shoot Command parameters
    // public double conveyorSpeed = 0.7;

    public double highShootSpeedTop = 1.0;
    public double highShootSpeedBottom = 0.6;

    public double lowShootSpeedTop = 0.3;
    public double lowShootSpeedBottom = 0.3;

    // paramters for autonmous
    public double collectAngle = 0;
    // todo fill in all the values below for autonomous cases
    // autonomous for 11.44 ft shot from right
    public double elevateROfGoal = 0.0; // elevate shooter for 11.44 ft shot
    public double shootTopROfGoal = 0.0; // top motor for 11.44 ft shot
    public double shootBotROfGoal = 0.0; // bot mottor for 11.44 ft shot

    // autonomous for 26.8 ft shot from back of trench (after collected third ball)
    public double elAutoBackTrench = 0.0;
    public double shootTopAutoBackTrench = 0.0;
    public double shootBotAutoBackTrench = 0.0;

    // autonomous for 10 ft shot from center
    public double elevateCenter = 40 * 1.1;
    public double shootTopCenter = 0.48 * 0.9;
    public double shootBotCenter = 0.48 * 0.9;

    // autonomous for 14.3 shot from delayed from right (moved back then shot)
    public double shootTopRDelay = 0.0;
    public double shootBotRDelay = 0.0;
    public double elRDelay = 0.0;

    // autonomous for 13.17 (now 13.8 -- same as Lob) shot from delayed from center
    public double shootTopCDelay = 0.48 * 0.9;
    public double shootBotCDelay = 0.48 * 0.9;
    public double elCDelay = 40 * 1.1;

    Config() {
        readConfig();
        // negative placed before ID to make nonfunctional but save number
        switch (robotType) {
        case Shooter:
            logf("Setup for parameters for Shooter\n");
            driveType = DriveType.OperatorTank;
            pcmID0 = true;
            elevatorID = -20;
            topShooterID = -8;
            bottomShooterID = 9;
            turretID = 21;
            shooterVelocityPID = false;
            driveRightFollow = 4;
            driveLeftFollow = 5;
            driveTicksPerInch = 2569 * 1.2;     
            actualTurnAngle45 = 23;    
            neoPixelsActive = false;
            ticksPerBall = 11 * (4096 * 56 * 2) / 7;
            ballConveyorID = 18;
            shooterElevateID = -18; // // Use 18 for testing with competition dart
            shooterElevateID = -6; // USe 6 for testing with competition dart
            intakeHorizontalID = 7;
            intakeVerticalID = 17;
            intakeConveryorID = 22;
            winchID = -8; // back motor
            winchID = -19; // Talon FX
            telescopeID = -18;
            limeLight = true;
            limeLightTurret = true;
            ledRing = false;
            telescopeCurrentLimit = 40;
            lidar = false;
            pigeon = false;
            wheelBase = 18.5;
            setShooeterPIDParameters();
            break;
        case SRXMini:
            driveType =  DriveType.OperatorTank;
            pcmID0 = true;
            elevatorID = 20;
            topShooterID = 8;
            bottomShooterID = 9;
            shooterVelocityPID = false;
            driveRightFollow = -1;
            driveLeftFollow = -1;
            neoPixelsActive = true;
            ticksPerBall = 11 * (4096 * 56 * 2) / 7;
            ballConveyorID = -18;
            shooterElevateID = -18; // // Use 18 for testing with competition dart
            shooterElevateID = -6; // USe 6 for testing with competition dart
            intakeHorizontalID = -18;
            turretID = 21;
            winchID = -8; // back motor
            winchID = -19; // Talon FX
            telescopeID = -18;
            limeLight = true;
            limeLightTurret = true;
            ledRing = false;
            telescopeCurrentLimit = 40;
            lidar = false;
            pigeon = false;

            if (testREVDriveonMini) {
                driveRight = 8;
                driveLeft = 9;
                topShooterID = -1;
                bottomShooterID = -1;
                driveTicksPerRevolution = 2000;
                driveTicksPerInch = 116 * 10;
            }
            setMiniPIDParameters();
            break;
        case RevMini:
            break;
        case RevChassis:
            wheelBase = 28;
            dualDriveMotors = true;
            // elevateShooterID = 8;
            // azimuthShooterID = 9;
            // bottomShooterID = 9;
            // topShooterID = 8;
            shooterVelocityPID = true;
            driveTicksPerInch = 116 * 5;
            break;
        case Competition:
            dualDriveMotors = true;
            driveType = DriveType.Tank;
            pcmID0 = true;
            pcmID1 = true;
            elevatorID = -20;
            topShooterID = 8;
            bottomShooterID = 9;
            // todo Shooter PID play with this
            shooterVelocityPID = true;
            neoPixelsActive = true;
            ticksPerBall = (10 + 1) * (4096 * 56 * 2) / 7;
            ballConveyorID = 18;
            shooterElevateID = 6;
            intakeHorizontalID = 14;
            winchID = 11;
            wheelBase = 21.875;
            wheelDiameter = 6.0;
            driveTicksPerRevolution = (int) (8450 * 12.0 / 14.0);
            driveTicksPerInch = driveTicksPerRevolution / (wheelDiameter * Math.PI);
            telescopeID = 10;
            enableCompressor = true;
            chameleonVision = true;
            ledRing = true;
            lidar = true;
            telescopeCurrentLimit = 2.5;
            setCompetitionPIDParameters();
            break;
        case SuitCase:
            driveType = DriveType.Tank;
            elevatorID = -1;
            enableCompressor = true;
            bottomShooterID = 9;
            topShooterID = 8;
            ballConveyorID = 18;
            setMiniPIDParameters();
            break;
        case SRXMiniV2:
            driveType = DriveType.OperatorMecanum;

        }
    }

    void readConfig() {
        String fileName = "/home/lvuser/deploy/parameters.txt";
        File fin = new File(fileName);
        if (!fin.exists()) {
            logf("File %s not found default values assumed\n", fileName);
            return;
        }
        try {
            BufferedReader br = new BufferedReader(new FileReader(fin));
            String line = null;
            while ((line = br.readLine()) != null) {
                line = line.trim();
                if (!line.startsWith("#")) {
                    String[] ar = line.split("=");
                    if (ar.length >= 2) {
                        String value = ar[1].trim();
                        if (value.contains("#")) {
                            value = value.split("#")[0].trim();
                        }
                        setVariable(ar[0].trim(), value);
                    }
                }
            }
            br.close();
        } catch (FileNotFoundException ex) {
            log("Unable to open file '" + fin.getName() + "'");
        } catch (IOException ex) {
            log("Error reading file '" + fin.getName() + "'");
        }
    }

    boolean setVariable(String variable, String value) {
        try {
            switch (variable.toLowerCase()) {
            case "defaultbrakemode":
                defaultBrakeMode = Boolean.valueOf(value);
                break;
            case "robottype":
                robotType = RobotType.valueOf(value);
                break;
            case "dualdrivemotors":
                dualDriveMotors = Boolean.valueOf(value);
                break;
            case "pcmid0":
                pcmID0 = Boolean.valueOf(value);
                break;
            case "pcmid1":
                pcmID1 = Boolean.valueOf(value);
                break;
            case "limelight":
                limeLight = Boolean.valueOf(value);
                break;
            case "wheelbase":
                wheelBase = Double.valueOf(value);
                break;
            case "opensightvision":
                opensightVision = Boolean.valueOf(value);
                break;
            default:
                logf("???? Warning Variable:%s not found in parms.txt file\n", variable);
                return false;
            }
        } catch (Exception ex) {
            logf("Unable to convert %s with a value of %s\n", variable, value);
            return false;
        }
        logf("Variable:%s value:%s\n", variable, value);
        return true;
    }

    void setMiniPIDParameters() {
        // positionPID = new SRXPID(60.0, .1, .0, 0.34, 2.5, 0.05, 0.5, 2);
        // positionPID = new SRXPID(1, 0.005, 10, 0, 2.5, 0.005 * 10, 0.5, 10);
        // positionPID = new SRXPID(2.50, .0002, 00, 0, 0, 0, 0, 2); // Worked for large
        // angle poor for small angle
        // positionPID = new PID("position", 3, .0002, 00, 0, 0, 0, 0, 0); // Pretty
        // good
        positionPID = new PID("position", 2.5, .0002, 0.0, 0.0, 0.0, 0.0, 0.0, 0, false); // Slow Down a bit 6/6/19
        velocityPID = new PID("velocity", 60, .15, .0, 0.34, 2.5, 0.05, 0.5, 2, false);
    }

    void setCompetitionPIDParameters() {
        // positionPID = new SRXPID(60.0, .1, .0, 0.34, 2.5, 0.05, 0.5, 2);
        // positionPID = new SRXPID(1, 0.005, 10, 0, 2.5, 0.005 * 10, 0.5, 10);
        positionPID = new PID("position", 2.50, .0002, 00, 0, 0, 0, 0, 2, false);
        velocityPID = new PID("velocity", 64, .1, .0, 0.34, 2.5, 0.05, 0.5, 2, false);
    }

    void setShooeterPIDParameters() {
        // positionPID = new SRXPID(60.0, .1, .0, 0.34, 2.5, 0.05, 0.5, 2);
        // positionPID = new SRXPID(1, 0.005, 10, 0, 2.5, 0.005 * 10, 0.5, 10);
        positionPID = new PID("position",.05, 0, 00, 0, 0, 0, 0, 2, false);
        velocityPID = new PID("velocity", 64, .1, .0, 0.34, 2.5, 0.05, 0.5, 2, false);
    }
}
