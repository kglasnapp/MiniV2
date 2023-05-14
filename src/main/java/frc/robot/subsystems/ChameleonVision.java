package frc.robot.subsystems;

import frc.robot.Robot;
import static frc.robot.utilities.Util.*;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

//import frc.robot.commands.DriveStraight.DriveMode;
//import frc.robot.commands.TurnTo.TurnMode;
//import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

// see https://chameleon-vision.readthedocs.io/en/latest/getting-started/networktables.html for the api definition

public class ChameleonVision extends SubsystemBase {
    NetworkTable table = NetworkTableInstance.getDefault().getTable("chameleon-vision").getSubTable("PSEYE");
    // MyCamName will vary depending on the name of your camera
    // Gets the MyCamName sub-table under the chamelon-vision table

    NetworkTableEntry isValid = table.getEntry("isValid");    // Whether or not a a target was found
    NetworkTableEntry targetPitch = table.getEntry("targetPitch");    // The vertical angle to the target in degrees.
    NetworkTableEntry targetYaw = table.getEntry("targetYaw");    // The horizontal angle to the target in degrees.
    NetworkTableEntry targetArea = table.getEntry("targetArea");    // The precentage of the target contour from the entire image
    NetworkTableEntry isDriverMode = table.getEntry("driverMode");    // Setting this to true enables the Driver Mode
    NetworkTableEntry currentPipeline = table.getEntry("pipeline");    // Change this to switch between pipelines
    NetworkTableEntry boundingWidth = table.getEntry("targetBoundingWidth");    // The width of the red rectangle in pixels
    NetworkTableEntry boundingHeight = table.getEntry("targetBoundingHeight");    // The height of the red rectangle in pixels
    NetworkTableEntry fittedWidth = table.getEntry("targetFittedWidth");    // The width of the blue rectangle in pixels
    NetworkTableEntry fittedHeight = table.getEntry("targetFittedHeight");    // The height of the blue rectangle in pixels
    NetworkTableEntry pose3D = table.getEntry("targetPose");    // The 3D target position as an array values are ordered by [y (meters),x (meters),angle (degrees)]

    public boolean valid = false;
    public double pitch = 0;
    public double yaw = 0;
    public double  area = 0;
    public double pipeline = 0;
    public double redWidth = 0;
    public double redHeight = 0;
    public double blueWidth = 0;
    public double blueHeight = 0;
    public boolean driverMode = false;
    public double[] pose = new double[3];
    private double[] defualtValue = new double[0];
    private boolean seekFinished = false;
    private boolean doLogs = true;
    private double driverCoolOff = 0;

    ////////////////////////////////////////////
    // Values for seek pid
    private long time = 0;
    private double bias = 0;
    private double error = 0;
    private double lastError = 0;
    private double lastIntegral = 0;
    private double derivative = 0;
    private double integral = 0;
    ////////////////////////////////////////////

    //private boolean cameraPipe = false;

    // sets the defualt pipeline value
    //private int pipelineStatus = -1; 
                                                        // status 0 = driverMode
                                                        // status 1 = trapezoidMode
                                                        // status 2 = humanPlayerMode

    public ChameleonVision() {
        if (Robot.config.chameleonVision) {
            isValid = table.getEntry("isValid");
            targetPitch = table.getEntry("targetPitch");
            targetYaw = table.getEntry("targetYaw");
            targetArea = table.getEntry("targetArea");
            isDriverMode = table.getEntry("driver_mode");
            currentPipeline = table.getEntry("pipeline");
            boundingWidth = table.getEntry("targetBoundingWidth");
            boundingHeight = table.getEntry("targetBoundingHeight");
            fittedWidth = table.getEntry("targetFittedWidth");
            fittedHeight = table.getEntry("targetFittedHeight");
            pose3D = table.getEntry("targetPose");
            log("ChameleonVision started\n");
/*
            if (pipelineStatus == 0) {
                driverMode = true;
                isDriverMode.setBoolean(driverMode);
            } else if (pipelineStatus == 1) {
                pipeline = 0;
                currentPipeline.setDouble(pipeline);
            } else if (pipelineStatus == 2) {
                pipeline = 1;
                currentPipeline.setDouble(pipeline);
            }
*/

        } else {
            log("Did not start ChameleonVision\n");
        }
    }

    public boolean tasks() {
     /*   if (Robot.oi.getRunwayLeft()) {
            visionDriveSeek(-1, false);
        } else if (Robot.oi.getRunwayRight()) {
            visionDriveSeek(1, false);
            */
        /*}else */
        if (Robot.oi.getSeekLeft()) { 
             //Seeks counterclockwise
            visionSeek(-1, false);
            driverCoolOff = Robot.count;
        } else if (Robot.oi.getSeekRight()){ //Seeks clockwise
            visionSeek(1, false);
            driverCoolOff = Robot.count;
        } else if (Robot.oi.setDriverCameraMode()) {
            setPipeline(-1);
        } else if (!driverMode) {
            if (Robot.count - driverCoolOff >= 2250) {
                setPipeline(-1);
                driverCoolOff = 0;
            }
        } else {
            return false;
        }
        return true;
    }

    public void setValues() {
        valid = (isValid.getBoolean(false));
        if (!valid) 
            return;
        pitch = round3(targetPitch.getDouble(0.0));
        yaw = round3(targetYaw.getDouble(0.0));
        area = round3(targetArea.getDouble(0.0));
        pipeline = currentPipeline.getDouble(0.0);
        redHeight = round3(boundingHeight.getDouble(0.0));
        redWidth = round3(boundingWidth.getDouble(0.0));
        blueHeight = round3(fittedHeight.getDouble(0.0));
        blueWidth = round3(fittedWidth.getDouble(0.0));
        driverMode = (isDriverMode.getBoolean(false));
        pose = pose3D.getDoubleArray(defualtValue);
    }

    public boolean getSeekFinished() {
        return seekFinished;
    }
    public void setSeekFinished(boolean stat) {
        seekFinished = stat;
    }

    /*
    public void visionSeek(int direction, boolean runwayAccess ) { // 1 = right, -1 =left
        setPipeline(1);
        double steeringAdjust;
        double  tolerance;
        //logf("visionSeek dir:%d run:%b\n", direction, runwayAccess);
        if (runwayAccess)
            tolerance = 4;
        else
            tolerance = 0.1;
        if (valid) {
            Robot.drive.setBrakeMode(true);
            double steerP = 0.05; // how hard to turn toward the target

            if (Math.abs(yaw) < 30)
                steerP *= 0.25;
            if (Math.abs(yaw) < 20)
                steerP *= 0.6;
            steeringAdjust = yaw * steerP;
            //logf("CHAMELEON YAW: %.1f ", yaw);
            logf("Target Found align x:%.1f steerP:%.1f\n", yaw, steerP);
            Robot.drive.setSpeed(steeringAdjust, steeringAdjust);
            if (Math.abs(yaw) <= tolerance) {
                Robot.drive.setSpeed(0, 0);
                seekFinished=true;
                log("STEER P IS GOOD\n");
                return;
            }
        } else {
            // We don't see the target, seek for the target by spinning in place
            steeringAdjust = 0.1*direction;//was .15
            if (Math.abs(steeringAdjust * 1000) / 1000.0 <  0.01 )  //set default error in limeLight to 4 stops ocilation
                return;
            logf("Target not found speed:%.1f actual R:%.1f, actual L:%.1f\n", steeringAdjust, Robot.drive.rightMotor.getSpeed(), Robot.drive.leftMotor.getSpeed());
            Robot.drive.setSpeed(steeringAdjust, steeringAdjust);
        }
    }
    */
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // todo TEST This is an experimental refined seek formula to be more smooth and easier to tune
    /*
    public void visionSeek(int direction, boolean runwayAccess ) { // 1 = right, -1 =left
        setPipeline(1);
        double steeringAdjust;
        double  tolerance;
        double a = 0.5;
        //logf("visionSeek dir:%d run:%b\n", direction, runwayAccess);
        if (runwayAccess)
            tolerance = 4;
        else
            tolerance = 0.5;
        if (valid) {
            Robot.drive.setBrakeMode(true);
            //double steerP = 0.03; // how hard to turn toward the target
            //steeringAdjust = yaw * steerP;

            steeringAdjust = (yaw*(0.0007297297297297*Math.abs(yaw)))*a;

            //logf("CHAMELEON YAW: %.1f ", yaw);
            logf("Target Found align x:%.1f steerP:%.1f\n", yaw, steeringAdjust);
            Robot.drive.setSpeed(steeringAdjust, steeringAdjust);
            if (Math.abs(yaw) <= tolerance) {
                Robot.drive.setSpeed(0, 0);
                seekFinished=true;
                log("STEER P IS GOOD\n");
                return;
            }
        } else {
            // We don't see the target, seek for the target by spinning in place
            steeringAdjust = 0.1*direction;//was .15
            if (Math.abs(steeringAdjust * 1000) / 1000.0 <  0.01 )  //set default error in limeLight to 4 stops ocilation
                return;
            logf("Target not found speed:%.1f actual R:%.1f, actual L:%.1f\n", steeringAdjust, Robot.drive.rightMotor.getSpeed(), Robot.drive.leftMotor.getSpeed());
            Robot.drive.setSpeed(steeringAdjust, steeringAdjust);
        }
    }
    */    
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    public void visionSeek(int direction, boolean runwayAccess ) { // 1 = right, -1 =left
        setPipeline(1);
        Robot.drive.setBrakeMode(true);
        double steeringAdjust;
        double  tolerance;
        //logf("visionSeek dir:%d run:%b\n", direction, runwayAccess);
        if (runwayAccess)
            tolerance = 4;
        else
            tolerance = 0;
        if (valid) {
            double steerP;
            double steerI;
            double steerD;
            //if (RobotController.getBatteryVoltage() > 12) {
                steerP = 0.01005101; // last stable: 0.01005101 
                steerI = 0.00000359; // last stable: 0.00000359
                steerD = 0.0754334; // last stable: 0.0754334 
            /*
            } else if (RobotController.getBatteryVoltage() > 10)
                steerP = 0.005;
                steerI = 0.0;
                steerD = 0.06;
            } else {
                steerP = 0.01
                steerI = 0.0
                steerD = 0.0
            }
            */
            error = yaw;
            derivative = (error - lastError) / (Robot.count - time);
            integral = lastIntegral + error * (Robot.count - time);
            steeringAdjust = ((steerP * error) + (steerI * integral) + (steerD * derivative) + bias);

            time = Robot.count;
            lastIntegral = integral;
            lastError = error;

            //logf("CHAMELEON YAW: %.1f ", yaw);
            logf("Target Found align x:%.1f steerP:%.1f\n", yaw, steerP);
            Robot.drive.setSpeed(steeringAdjust, steeringAdjust);
            if (Math.abs(yaw) <= tolerance) {
                Robot.drive.setSpeed(0, 0);
                seekFinished=true;
                log("STEER P IS GOOD\n");
                return;
            }
        } else {
            // We don't see the target, seek for the target by spinning in place
            steeringAdjust = 0.15*direction;//was .15
            if (Math.abs(steeringAdjust * 1000) / 1000.0 <  0.01 )  //set default error in limeLight to 4 stops ocilation
                return;
            logf("Target not found speed:%.1f actual R:%.1f, actual L:%.1f\n", steeringAdjust, Robot.drive.rightMotor.getSpeed(), Robot.drive.leftMotor.getSpeed());
            Robot.drive.setSpeed(steeringAdjust, steeringAdjust);
        }
    }

    public void visionOffSetSeek(int direction, double deltaYaw) { // 1 = right, -1 =left
        double steeringAdjust;
        double drivingSpeed = 1.0;
        double tolerance = 0.3;
        //Robot.drive.setBrakeMode(true);
        double steerP = 0.02; // how hard to turn toward the target
        if (targetIsValid()) {
            steeringAdjust =  (deltaYaw * steerP);
            Robot.drive.setSpeed(-(drivingSpeed-steeringAdjust), drivingSpeed - steeringAdjust);
            if (Math.abs(deltaYaw) <= tolerance) {
                Robot.drive.setSpeed(0, 0);
                seekFinished=true;
                log("STEER P IS GOOD\n");
                return;
            }
        } else {
            // We don't see the target, seek for the target by spinning in place
            steeringAdjust = 0.5*direction;
            if (Math.abs(steeringAdjust * 1000) / 1000.0 <  0.01 )  //set default error in limeLight to 4 stops ocilation
                return;
            Robot.drive.setSpeed(steeringAdjust, steeringAdjust);
        }
    }

/*
    public void visionDriveSeek(int direction, boolean runwayAccess ) { // 1 = right, -1 =left
        double steeringAdjust;
        double  tolerance;
        double driveSpeed = 1.0;
        logf("visionSeek dir:%d run:%b\n", direction, runwayAccess);
        if (runwayAccess)
            tolerance = 4;
        else
            tolerance = 2;
        if (valid) {
            double steerP = 0.02; // how hard to turn toward the target
            if (Math.abs(yaw) < 2)
                steerP *= 3;
            if (Math.abs(yaw) < 0.1)
                steerP *= 0.2;
            steeringAdjust = yaw * steerP;
            logf("Target Found align x:%.1f steerP:%.1f\n", yaw, steerP);
            Robot.drive.setSpeed(-(driveSpeed - steeringAdjust), driveSpeed - steeringAdjust);
            if (Math.abs(yaw) <= tolerance) {
                Robot.drive.setSpeed(0, 0);
                log("STEER P IS GOOD\n");
                return;
            }
        } else {
            // We don't see the target, seek for the target by spinning in place
            steeringAdjust = 0.5*direction;
            if (Math.abs(steeringAdjust * 1000) / 1000.0 <  0.01 )  //set default error in limeLight to 4 stops ocilation
                return;
            logf("Target not found speed:%.1f actual R:%.1f, actual L:%.1f\n", steeringAdjust, Robot.drive.rightMotor.getSpeed(), Robot.drive.leftMotor.getSpeed());
            Robot.drive.setSpeed(steeringAdjust, steeringAdjust);
        }
    }
*/
    //Returns target distance in Inches
    public double targetDistance() {
        double d = 0;
        if(valid) {
            // 39.37 converts meters to in
            d = (pose[0]*39.37);
            if (doLogs)
                logf("CHAMELEON TARGET DISTANCE INCHES: %.1f\n", d);
        }
        return d;
    }

    public double targetDistanceX() {
        double d = 0;
        if(valid)
            // 39.37 converts meters to in
            d = ((pose[0]*39.37)*Math.sin(Math.toRadians(Math.abs(pose[2]))));
            if (doLogs)
                logf("CHAMELEON TARGET DISTANCE: %.1f\n", d);
        return d;
    }

    public double targetDistanceY() {
        double d = 0;
        if(valid)
            // 39.37 converts meters to in
            d = ((pose[0]*39.37)*Math.cos(Math.toRadians(Math.abs(pose[2]))));
            if (doLogs)
                logf("CHAMELEON TARGET DISTANCE Y: %.1f\n", d);
        return d;
    }

    // this returns the angle to the robot fron the target in degrees from -45 to 45 
    // not the angle to the target, the angle from the target!
    public double targetAngle() {
        double a = 0;
        if(valid) {
            a = Math.abs(pose[2]);
            if (doLogs)
                logf("CHAMELEON TARGET ANGLE DEGREES: %.1f\n", a);
        }
        return a;
    }

    public double targetYaw() {
        return yaw;
    }

    public boolean targetIsValid() {
        return valid;
    }

    public void setPipeline(double number) {
        currentPipeline.setDouble(number);
        //isDriverMode.setBoolean(true);
    }

    public void postData() {
        SmartDashboard.putNumber("Raw Target Distance X", round3(pose[0]) );
        SmartDashboard.putNumber("Raw Target Distance Y", round3(pose[1]));
        SmartDashboard.putNumber("Raw Target Angle", round3(pose[2]));
    }

    // public void teleopPeriodic() {
    //     setValues();
    //     if (Robot.leds != null)
    //         Robot.leds.limeLightLed.set(valid); // Set the led to indicate that we have a valid vision target
    //     if (Robot.count % 25 == 20) 
    //         postData();
    // }

    @Override
    public void periodic() {
        setValues();
        tasks();
    }
}