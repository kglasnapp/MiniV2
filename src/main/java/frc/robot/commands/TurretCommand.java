/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import static frc.robot.utilities.Util.logf;
//import frc.robot.Robot;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;

/**
 * An example command. You can replace me with your own command.
 */
public class TurretCommand extends CommandBase {
  double requestedAngle = 0;

  public enum TurretState {
    Idle, MoveLeft, MoveRight, MoveHome, Homed
  };

  TurretState state = TurretState.Idle;
  Timer timer = new Timer();
  double timeOut;

  public TurretCommand(double angle) {
    // Move the turret to the indicated angle
    // If the angle is -1 do a limit test and then move to the 0 degrees
    this.requestedAngle = angle;
    // Use requires() here to declare subsystem dependencies
    addRequirements(Robot.turretShooter);
  }

  // Called just before this Command runs the first time
  @Override
  public void initialize() {
    logf("Turret Command started angle:%.2f\n", requestedAngle);
    if (requestedAngle != -1) {
      Robot.turretShooter.turretSetAngle(requestedAngle, false);
      timeOut = 5;
    } else {
      // Home the turret
      state = TurretState.MoveLeft;
      Robot.turretShooter.turretMoveToLimit(-179);
      timeOut = 30;
    }
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  public void execute() {
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  public boolean isFinished() {
    int pos = Robot.turretShooter.getTurretPosition();
    boolean rLim = Robot.turretShooter.rightLimitTurret();
    boolean lLim = Robot.turretShooter.leftLimitTurret();
    double angle = Robot.turretShooter.getTurretAngle();
    SmartDashboard.putNumber("TurretPos", pos);
    logf("Turret State:%s Angle:%.1f Postion:%d %s Limit <L:%b,r:%b>\n", state.toString(), angle, pos,
        Robot.turretShooter.getTurretVCS(),
        lLim, rLim);
    if (timer.hasElapsed(timeOut)) {
      logf("????? Turret Command Timeout\n");
      return true;
    }
    if (requestedAngle != -1) {
      if (Robot.turretShooter.isTurretFinished()) {
        logf("+++++ Turrent at selected position %d\n", pos);
        return true;
      }
      return false;
    }
    if (state == TurretState.MoveLeft) {
      if (lLim) {
        Robot.turretShooter.setTurretPositionAtLeftLimit();
        Robot.turretShooter.turretMoveToLimit(179);
        state = TurretState.MoveRight;

      }
    }
    if (state == TurretState.MoveRight) {
      if (rLim) {
        Robot.turretShooter.turretMoveToLimit(0);
        state = TurretState.MoveHome;
      }
    }
    if (state == TurretState.MoveHome) {
      if (Robot.turretShooter.isTurretFinished()) {
        state = TurretState.Homed;
        logf("+++++ Homing Complete Angle:%.1f\n", angle);
        return true;
      }
    }
    return false;
  }

  // Called once after isFinished returns true
  @Override
  public void end(boolean interrupted) {
    int pos = Robot.turretShooter.getTurretPosition();
    logf("+++++ Turret Command Complete Position:%d\n", pos);
    Robot.turretShooter.turretSetSpeed(0);
  }

}
