/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import static frc.robot.utilities.Util.logf;
import frc.robot.Robot;

/**
 * An example command. You can replace me with your own command.
 */
public class ZeroYaw extends CommandBase {

    public ZeroYaw() {
        // Use requires() here to declare subsystem dependencies

    }

    // Called just before this Command runs the first time
    @Override
    public void initialize() {
        logf("Zero the yaw current yaw:%.2f\n", Robot.yaw);
        if (Robot.yawSensor != null)
            Robot.yawSensor.zeroYaw();
    }

    // Called repeatedly when this Command is scheduled to run
    @Override
    public void execute() {
    }

    // Make this return true when this Command no longer needs to run execute()
    @Override
    public boolean isFinished() {
        return true;
    }

    // Called once after isFinished returns true
    @Override
    public void end(boolean interrupted) {
    }

}
