/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import static frc.robot.utilities.Util.logf;
import frc.robot.Robot;
import edu.wpi.first.wpilibj.Timer;

/**
 * An example command. You can replace me with your own command.
 */
public class ShootAllBalls extends CommandBase {
    enum States {
        Idle, Startup, ShooterAtSpeed, BallFeedActive, WaitForShutdown
    };

    States state = States.Idle;
    boolean targeting = true; // Set to true if you want to target before shooting
    boolean targetFound = true;
    Timer timer = new Timer();

    // The functions performed are:
    // If targeting active -- aim the turret at the the target
    // Start the shooter -- move to state
    // Wait for the shooter speed to come up
    // If camera sees target then start the ball feed motor

    public ShootAllBalls(boolean targeting) {
        // Use requires() here to declare subsystem dependencies
        addRequirements(Robot.turretShooter);
        this.targeting = targeting;
    }

    // Called just before this Command runs the first time
    @Override
    public void initialize() {
        logf("+++++ Start Shooting All Balls until Button Released\n");
        state = States.Startup;
        if (!targeting) {
            targetFound = true;
        }
        timer = new Timer();
        timer.start();
    }

    // Called repeatedly when this Command is scheduled to run
    @Override
    public void execute() {

    }

    // Make this return true when this Command no longer needs to run execute()
    @Override
    public boolean isFinished() {
        if (Robot.count % 10 == 0) {
            logf("Shooter State %s\n", state.toString());
        }
        if (timer.hasElapsed(8)) {
            logf("!!!! Shooter Logic Timed out\n");
            return true;
        }
        targeting = Robot.oi.noTargeting();
        boolean button = Robot.oi.opShootBalls();
        boolean limeLightValid = Robot.limeLightVision.valid;
        double angle = Robot.limeLightVision.x;
        double actualSpeed = Math.abs(Robot.turretShooter.getBottomActualSpeed());
        SmartDashboard.putNumber("ShootSp", actualSpeed);
        double position = Robot.turretShooter.getBottomPositon();
        double turretAngle = Robot.turretShooter.getTurretAngle();
        logf("Shoot state:%s speed:%.1f position:%.1f llValdid:%b llAng:%.1f turAng:%.1f button:%b\n", state,
                actualSpeed, position, limeLightValid, angle, turretAngle, button);
        if (targeting && limeLightValid) {
            Robot.turretShooter.turretSetAngle(angle - turretAngle, true);
            if (Math.abs(angle) < .5)
                targetFound = true;
        }
        if (targeting && !limeLightValid) {
            Robot.turretShooter.aim.aimPeriodic();
        }
        if (state == States.Startup) {
            // Start shooter motor
            Robot.turretShooter.setShootSpeedActual(-0.8);
            if (actualSpeed > 40000) {
                state = States.ShooterAtSpeed;
            }
        }
        // targetFound = true;
        if (state == States.ShooterAtSpeed && targetFound) {
            // Start ball Intake motor
            Robot.turretShooter.ballIntakeOn();
        }
        if (!button) {
            // Stop shooting -- trigger released
            state = States.WaitForShutdown;
            return true;
        }

        return false;
    }

    // Called once after isFinished returns true
    @Override
    public void end(boolean interrupted) {
        logf("----- Shoot All Balls complete\n");
        Robot.turretShooter.setShootSpeed(0);
        Robot.turretShooter.ballIntakeOff();
        // Robot.turretShooter.aim.aimStop();
        // Robot.turretShooter.turretSetAngle(0, false);
        state = States.Idle;
        ;
    }

}
