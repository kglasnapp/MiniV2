/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
//import frc.robot.Robot;
import frc.robot.Robot;

/**
 * An example command. You can replace me with your own command.
 */
public class MoveBallConveyor extends CommandBase {
    double speed = 0.0;

    public MoveBallConveyor(double speed) {
        this.speed = speed;
    }

    // Called just before this Command runs the first time
    @Override
    public void initialize() {
        Robot.turretShooter.setBallConveyorSpeed(speed);

    }

    // Called repeatedly when this Command is scheduled to run
    @Override
    public void execute() {
    }

    // Make this return true when this Command no longer needs to run execute()
    @Override
    public boolean isFinished() {
        if (Robot.oi.moveBallConveyorButton()) {
            return false;
        }
        Robot.turretShooter.setBallConveyorSpeed(0);
        return true;
    }

    // Called once after isFinished returns true
    @Override
    public void end(boolean interrupted) {
    }

}
