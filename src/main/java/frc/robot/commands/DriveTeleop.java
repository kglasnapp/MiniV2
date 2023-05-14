
package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
//import frc.robot.commands.DriveStraight.DriveMode;
import frc.robot.subsystems.Drive;
import frc.robot.utilities.PIDController;
import frc.robot.utilities.RunningAverage;
import frc.robot.utilities.Util;

// Buttons used my drive teleop
// Right Joy 5 -- turn 180
// Right Joy 7 -- turn 180 PID based
// Operator POV for different turns
// Left Joy Trigger - Drive Straight

public class DriveTeleop extends CommandBase {
    PIDController turnPID = new PIDController(.05, 0, 0, 0);
    private double myYaw;
    private Double targetYaw = null;
    private RunningAverage runAvg = new RunningAverage(10);
    private Drive drive;

    public DriveTeleop(Drive drive) {
        this.drive = drive;
        addRequirements(this.drive);
        Util.logf("+++++++++++ Start drive Telop ++++++++++++++\n");
    }

    @Override
    public void execute() {
        myYaw = Robot.yaw;
        runAvg.add(myYaw);
        // Do turns based upon buttons being pressed
        // if (turnTo180())
        // return;
        // if (turnToAnglePIDBased(180))
        // return;
        // if (turnToPovPIDBased())
        // return;
        if (driveStraight())
            return;
        // if (Robot.oi.getLimelightSeekLeft() || Robot.oi.getLimelightSeekRight()) //
        // if button five on right joy pressed
        // return;
        // if (Robot.oi.getLimelightTracking()) {
        // return;
        // }

        // Perform actual Drive task based upon joy sticks
       drive.driveRobot();
    }

    @Override
    public boolean isFinished() {
        return false; // This command will never finish
    }

    public boolean driveStraight() {
        // Set Target Yaw if Right Trigger Button Pressed
        double adjustment = .045;
        if (Robot.oi.driveStraightPressed()) {
            Util.logf("++++ Drive Straight Set Target Yaw %.1f adjustment: %.5f\n", myYaw, adjustment);
            targetYaw = Robot.yaw;
        }
        if (Robot.oi.driveStraightReleased()) {
            Util.logf("----- Drive Straight Complete yaw:%.1f target:%.1f\n", myYaw, targetYaw);
            targetYaw = null;
        }
        if (targetYaw != null) {
            // Use the difference between initial yaw and yaw to determine a correction
            double diff = Robot.yaw - targetYaw;
            if (diff > 10) {
                Util.logf("!!!!! Drive straight Delta too positive diff:%.1f yaw:%.1f target:%.3f\n", diff, Robot.yaw,
                        targetYaw);
                diff = 5;
            }
            if (diff < -10) {
                Util.logf("!!!!! Drive straight Delta too negiative diff:%.1f yaw:%.1f target:%.3f\n", diff, Robot.yaw,
                        targetYaw);
                diff = -5;
            }
            // Sum the values from each of joy sticks
            // double joy = Robot.oi.driveStraightSpeed();
            double joy = Robot.drive.getRightSpeed() - Robot.drive.getLeftSpeed();
            // If joy stick is too fast reduce the speed
            if (joy > .5)
                joy = .5;
            if (joy < -.5)
                joy = -.5;

            // Determine the correction factor for driving based upon the yaw
            double factor = diff * Math.abs(joy) * adjustment;
            if (Robot.count % 3 == 0) // Log every 60 ms while in drive straight
                Util.logf("Drive straight factor:%.5f Yaw:%.1f runAvg:%.3f target:%.1f joy:%.3f <%.3f:%.3f>\n", factor,
                        myYaw, runAvg.getAverage(), targetYaw, joy, Robot.oi.rightJoySpeed(), Robot.oi.leftJoySpeed());
            // Drive the motors based upon the correction factor
            if (!Robot.drive.getStopForwardMotion()) {
                Robot.drive.setSpeed(joy - factor, -joy - factor);
            } else {
                Robot.drive.setSpeed(0.0, 0.0);
            }
            return true;
        }
        return false;
    }

    public boolean turnTo180() {
        if (Robot.oi.turn180()) {
            double steerP = 0.6; // how hard to turn toward the target
            Util.logf("Turn 180 yaw:%.1f steerP:%.3f\n", myYaw, steerP);
            if (Math.abs(myYaw) < 170) {
                Robot.drive.setSpeed(steerP, steerP);
                return true;
            }
        }
        return false;
    }

    public boolean turnToAnglePIDBased(double angle) {
        if (Robot.oi.turnToPIDBased()) {
            // yaw is between -180 and +180
            // myYaw will take yaw and convert it to 0 to 360
            myYaw = (myYaw < 0) ? myYaw + 360 : myYaw;
            // Call the PID to determine the error
            double turnSpeed = turnPID.updatePID(myYaw, angle);
            Util.logf("Turn to an angle:%.1f yaw:%.1f speed:%.3f\n", angle, myYaw, turnSpeed);
            Robot.drive.setSpeed(turnSpeed, turnSpeed);
            return true;
        }
        return false;
    }

    public boolean turnToPovPIDBased() {
        int pov = Robot.oi.getPOV();
        if (pov != -1) {
            myYaw = (myYaw < 0) ? myYaw + 360 : myYaw;
            if (pov > 350 || pov < 10) {
                myYaw = Robot.yaw;
                pov = (int) Util.normalizeAngle(pov);
            }
            double turnSpeed = turnPID.updatePID(myYaw, pov);
            Util.logf("Turn %d yaw:%.1f speed:%.3f\n", pov, myYaw, turnSpeed);
            Robot.drive.setSpeed(turnSpeed, turnSpeed);
            return true;
        }
        return false;
    }

    // For Mecanum
    // For temporary mecanum: //todo define in Drive sub?
    /*
     * public void strafe() { while ( ( ( gamepad.getRawAxis(4) > 0.2) || (
     * gamepad.getRawAxis(4) < -0.2 ) ) && ( (gamepad.getRawAxis(5) < 0.1) &&
     * (gamepad.getRawAxis(5) > -0.1 ) ) ) { //strafe left
     * motorLeftBack.setInverted(true); motorLeftFront.setInverted(true);
     * motorRightFront.set( gamepad.getRawAxis(4) ); motorLeftFront.set( -1 *
     * gamepad.getRawAxis(4)); motorRightBack.set( -1 * gamepad.getRawAxis(4));
     * motorLeftBack.set( gamepad.getRawAxis(4));
     * 
     * if ( ( gamepad.getRawAxis(0) > 0.1 ) || (gamepad.getRawAxis(0) < -0.1) ) {
     * return; } } } moved to Drive
     */
}
