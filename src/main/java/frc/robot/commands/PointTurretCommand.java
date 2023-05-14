/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import static frc.robot.utilities.Util.logf;
import edu.wpi.first.wpilibj.Timer;

/**
 * An example command. You can replace me with your own command.
 */
public class PointTurretCommand extends CommandBase {
    int validCount = 0;
    int invalidCount = 0;
    Timer timer = new Timer();

    public PointTurretCommand() {
        // Use requires() here to declare subsystem dependencies
        // requires(Robot.m_subsystem);
        addRequirements(Robot.turretShooter);
    }

    // Called just before this Command runs the first time
    @Override
    public void initialize() {
        logf("PointTurretCommand\n");
        validCount = 0;
        invalidCount = 0;
       timer.start();
    }

    // Called repeatedly when this Command is scheduled to run
    @Override
    public void execute() {
    }

    // Make this return true when this Command no longer needs to run execute()
    @Override
    public boolean isFinished() {
        if(timer.hasElapsed(3)) {
            logf("????? Point Turret Timeout\n");
            return true;
        }
        if(invalidCount > 10) {
            logf("????? LimeLight Invalid for a number of cycles\n");
            return true;
        }
        if (Robot.limeLightVision.valid) {
            invalidCount = 0;
            double limelightAngle = Robot.limeLightVision.x;
            double turretAngle = Robot.turretShooter.getTurretAngle();
            logf("LimeLight point Turret to turret angle:%.1f limelight angle:%.2f validCount:%d\n", turretAngle,
                    limelightAngle, validCount);
            Robot.turretShooter.turretSetAngle(limelightAngle - turretAngle, false);
            if (Math.abs(limelightAngle) < .2)
                validCount++;
            else
                validCount = 0;
            if (validCount > 4)
                return true;
        } else {
            logf("LimeLight Data not valid\n");
            invalidCount++;
            validCount = 0;
        }
        return false;
    }

    // Called once after isFinished returns true
    @Override
    public void end(boolean interrupted) {
        logf("----- Point Turret End valid:%b x:%.2f\n", Robot.limeLightVision.valid, Robot.limeLightVision.x);
    }
}
