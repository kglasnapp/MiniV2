package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drive;
import frc.robot.utilities.Util;
import frc.robot.Robot;
import edu.wpi.first.wpilibj.Timer;

/**
 * An example command. You can replace me with your own command.
 */
public class DriveStraight extends CommandBase {

    public enum DriveMode {
        RELATIVE_INCHES, INCHES_FROM_BACK_SENSOR, INCHES_FROM_FRONT_SENSOR, UNTIL_OBSTACLE, INCHES_VIA_LIDAR
    }

    private double distance;
    private final double origDistance;
    private final DriveMode mode;

    private double speed;
    private final Drive drive;

    private double initialYaw;
    private double actualDistance;
    private Timer timer = new Timer();
    private double timeOut;

    public DriveStraight(DriveMode mode, double distance, double speed, double timeOut) {
        //requires(Robot.drive);
        this.distance = distance;
        this.origDistance = distance;
        this.mode = mode;
        this.speed = speed;
        this.timeOut = timeOut;
        drive = Robot.drive;
        Util.logf("Setup a Drive Straight cmd dist:%.1f speed:%.1f\n", distance, speed);
        addRequirements(Robot.drive);
    }

    // Called just before this Command runs the first time
    @Override
    public void initialize() {
        //requires(Robot.drive);
        initialYaw = Robot.yaw;
        drive.setBrakeMode(false);
        drive.setInitialPosition();
        if (mode == DriveMode.INCHES_FROM_FRONT_SENSOR) {
            distance = Robot.frontDist - origDistance;
        }
        if (mode == DriveMode.INCHES_FROM_BACK_SENSOR) {
            distance = origDistance - Robot.rearDist;
        }
        if(mode == DriveMode.INCHES_VIA_LIDAR) {
            distance = Robot.lidar.getDistanceInches() - origDistance;
        }
        speed = Math.copySign(speed, distance);
        double lidarDist = 0;
        if(Robot.config.lidar)
        lidarDist = Robot.lidar.getDistanceInches();
        Util.logf("++++ Drive Straight cmd mode:%s request dist:%.1f distance:%.1f Yaw:%.1f Front:%.1f Rear:%.1f Lidar:%.1f\n", mode,
                origDistance, distance, initialYaw, Robot.frontDist, Robot.rearDist, lidarDist);
        timer.start();
    }

    // Called repeatedly when this Command is scheduled to run
    @Override
    public void execute() {
    }

    // Make this return true when this Command no longer needs to run execute()
    @Override
    public boolean isFinished() {
        // Determine the yaw error
        double yawError = Util.normalizeAngle(Robot.yaw - initialYaw);
        double yawFactor = (yawError * 0.003);
        actualDistance = drive.getDistanceInches(); // Get actual distance from encoders
        double newSpeed = getNewSpeed(speed, actualDistance);
        //double newSpeed = speed;
        drive.setSpeed(-newSpeed - yawFactor, newSpeed - yawFactor);
        Util.loginfo("Drive Straight cmd time:%.1f speed:%.2f dist:<%.1f,%.1f> yaw:<%.1f,E:%.1f> \n", timer.get(),
                newSpeed, distance, actualDistance, Robot.yaw, yawError);
        if (timer.hasElapsed(timeOut)) {
            Util.log("???? DriveStraight TimeOut");
            return true;
        }
        // Stop when distance is at less than 1.5 inches
        if (Math.abs(actualDistance) > (Math.abs(distance) - 1.5)) {
            return true;
        }
        return false;
    }

    // Called once after isFinished returns true
    @Override
    public void end(boolean interrupted) {
        drive.setSpeed(0.0, 0.0);
        drive.setBrakeMode(Robot.config.defaultBrakeMode);
        Util.logf("---- Drive Straight cmd End mode:%s Dist<%.1f,%.1f,%.1f> Yaw<%.1f,%.1f> Elapsed:%.2f\n", mode, distance,
                actualDistance, actualDistance - distance, Robot.yaw, Robot.yaw - initialYaw,
                timer.get());
    }


    // Determine speed based upon acceleration and deceleration
    double acceleration = .4; // value is number of seconds to achieve full speed
    double deceleration = 12; // Value is the number of inches when to start slowing down

    double getNewSpeed(double speed, double actualDistance) {
        double t = timer.get();
        if (t < acceleration) {
            return speed * t / acceleration;
        }
        double remainingDistance = Math.abs(actualDistance - distance) + 2;
        if (remainingDistance < deceleration) {
            return speed * remainingDistance / deceleration;
        }
        return speed;
    }

}