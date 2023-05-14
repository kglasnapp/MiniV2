/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.LimeLightVision;
import frc.robot.utilities.*;
import edu.wpi.first.wpilibj.Timer;

/**
 * An example command. You can replace me with your own command.
 */
public class LogLimeLightDataCommand extends CommandBase {
    public enum State {
        LINEUP, COLLECT, DRIVE
    }

    State state = State.LINEUP;
    int myCount = 0;
    LimeLightVision vision;
    Drive drive;
    Timer timer = new Timer();

    public LogLimeLightDataCommand() {
        // Use requires() here to declare subsystem dependencies
        addRequirements(Robot.drive);
    }

    // Called just before this Command runs the first time
    @Override
    public void initialize() {
        vision = Robot.limeLightVision;
        drive = Robot.drive;
        Util.log("Start Log Limelight Data");
        Robot.limeLightVision.createLimeLightFile();
        state = State.LINEUP;
        timer.start();
    }

    double lastTime = 0;

    // Make this return true when this Command no longer needs to run execute()
    @Override
    public boolean isFinished() {
        double startTime = timer.get();
        // Util.logf("State:%s time:%.1f\n", state, startTime);
        if (state == State.LINEUP) {
            vision.limeLightLineUp();
            if (startTime > .2) {
                state = State.COLLECT;
                Util.log("Save Data in LineUp state");
            }
            lastTime = startTime;
            return false;
        }
        if (state == State.COLLECT) {
            if (lastTime + .1 > startTime) {
                vision.saveLimeLightData(startTime);
                Util.logf("Save Data in COLLECT state dist:%.1f\n", Robot.frontDist);
                state = State.DRIVE;
                drive.setInitialPosition();
                if (Robot.frontDist < 15) {
                    Util.log("LogLidarData -- finished");
                    return true;
                }
            }
            return false;
        }
        if (state == State.DRIVE) {
            double steerCommand = 0.0;
            final double STEER_K = 0.03; // how hard to turn toward the target
            steerCommand = vision.x * STEER_K;
            drive.setSpeed(-.2 + steerCommand, .2 + steerCommand);
            double dist = Math.abs(drive.getDistanceInches());
            // Util.logf("Drive forward dist:%.1f\n", dist);
            if (dist > 6) {
                state = State.COLLECT;
                lastTime = startTime;
            }
            return false;
        }
        return true;
    }

    // Called once after isFinished returns true
    @Override
    public void end(boolean interrupted) {
        Robot.limeLightVision.closeLimeLightFile();
    }
}
