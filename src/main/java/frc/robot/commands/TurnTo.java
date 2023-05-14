/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.utilities.Util;
import frc.robot.Robot;
import edu.wpi.first.wpilibj.Timer;

/**
 * Command to perform a turnTo function Various modes exist ABSOLUTE, RELATIVE,
 * GRID_CW, GRID_CCW, VISION User specifies the mode and desired angle
 */

public class TurnTo extends CommandBase {

  public enum TurnMode {
    ABSOLUTE, RELATIVE, GRID_CW, GRID_CCW, VISION
  }

  private double targetAngle;
  private double requestedAngle;
  private TurnMode mode;
  private final double gridError = 3.0;

  private int testTicks = 0;
  private boolean testTurnTo = false;

  private double initialDelta = 0;
  private int shortTurnCount = 0;
  private int count = 0;
  Timer timer = new Timer();

  public TurnTo(TurnMode mode, double angle) { // positive angle CW, negative angle CCW
    addRequirements(Robot.drive);
    this.requestedAngle = angle;
    this.mode = mode;
    Util.loginfo("Create TurnTo %.1f %s\n", angle, mode);
  }

  public TurnTo(TurnMode mode) {
    this.requestedAngle = 90;
    this.mode = mode;
    Util.loginfo("Create TurnTo no angle  %s\n", mode);
    addRequirements(Robot.drive);
  }

  // Called just before this Command runs the first time
  @Override
  public void initialize() {
    timer.start();
    count = 0;
    // Defaults to turn Absolute Angle
    targetAngle = requestedAngle;

    if (mode == TurnMode.RELATIVE)
      targetAngle = Robot.yaw + targetAngle;

    if (mode == TurnMode.GRID_CCW)
      targetAngle = Math.floor((Robot.yaw - gridError) / targetAngle) * targetAngle;

    // Math.floor((currentYaw + gridError) / 90) * 90 + 90
    if (mode == TurnMode.GRID_CW)
      targetAngle = Math.floor((Robot.yaw + gridError) / targetAngle) * targetAngle + targetAngle;

    targetAngle = Util.normalizeAngle(targetAngle);
    Util.logf("++++ TurnTo mode: %s requested angle:%.1f target angle:%.1f current yaw:%.1f\n", mode, requestedAngle,
        targetAngle, Robot.yaw);
    // Robot.drive.setBrakeMode(true);

    initialDelta = Util.normalizeAngle(Util.normalizeAngle(targetAngle) - Util.normalizeAngle(Robot.yaw));
    if (Math.abs(initialDelta) < 9) {
      shortTurnCount = 2;
    }

    if (testTurnTo) {
      testTicks = (int) (4000 * requestedAngle);
      Robot.drive.rightMotor.setActualPosition(-testTicks, true);
      Robot.drive.leftMotor.setActualPosition(-testTicks, true);
    }

  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  public void execute() {
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  public boolean isFinished() {
    count++;
    if (timer.hasElapsed(3)) {
      Util.logf("??? TurnTo timed out requested:%.1f yaw:%.1f error:%.1f\n", targetAngle, Robot.yaw,
          Util.normalizeAngle(Util.normalizeAngle(targetAngle) - Util.normalizeAngle(Robot.yaw)));
      return true;
    }
    int rightEncoder = Robot.drive.rightEncoder();
    int leftEncoder = Robot.drive.leftEncoder();
    if (testTurnTo) {
      long testError = Math.abs(rightEncoder - testTicks)
          + Math.abs(leftEncoder - testTicks);
      Util.logf("TurnTo time:%f yaw:%.1f target:%d error:%d enc<%d,%d>\n", this.timer.get(), Robot.yaw,
          testTicks, testError, rightEncoder, leftEncoder);
      if (testError < 30)
        return true;
      return false;
    }

    if (Math.abs(initialDelta) < 9) {
      return shortTurn(initialDelta);
    }

    // Use the difference between initial yaw and yaw to determine a correction
    double error = Util.normalizeAngle(Util.normalizeAngle(targetAngle) - Util.normalizeAngle(Robot.yaw));
    int ticks = (int) ((error / 360.0) * Robot.config.wheelBase * Math.PI * Robot.config.driveTicksPerInch);
    // Drive the motors based upon the correction factor
    if (count %  20 == 1) {
      ticks = (int) ((ticks * 45.0) / Robot.config.actualTurnAngle45);
      Robot.drive.rightMotor.setActualPosition(ticks - rightEncoder, true);
      Robot.drive.leftMotor.setActualPosition(ticks - leftEncoder, true);
      Util.logf("Adjust ticks:%d\n", ticks);
    }
    if (count % 1 == 0) {
      Util.logf("TurnTo time:%f yaw:%.1f target:%.1f error:%.1f ticks:%d enc r:%d,l:%d\n", this.timer.get(),
          Robot.yaw, targetAngle, error, ticks,rightEncoder, leftEncoder);
    } 

    if (Math.abs(error) < 1.1) {
      Util.logf("*** TurnTo angle achieved, error:%.1f target:%.1f yaw:%.1f\n", error, targetAngle, Robot.yaw);
      return true;
    }
    return false;
  }

  // Called once after isFinished returns true
  @Override
  public void end(boolean interrupted) {
    Robot.drive.setSpeed(0, 0);
    Robot.drive.setDefaultBrakeMode();
    Util.logf("---- TurnTo end target angle:%.2f yaw:%.2f\n", targetAngle, Robot.yaw);
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run

  boolean shortTurn(double angle) {
    shortTurnCount--;
    double speed = 0.3;
    if (angle < 0)
      speed = -speed;
    double error = Util.normalizeAngle(Util.normalizeAngle(targetAngle) - Util.normalizeAngle(Robot.yaw));
    Util.logf("Short turn error:%.1f count:%d speed:%.1f\n", error, shortTurnCount, speed);
    if (shortTurnCount < 0)
      return true;
    Robot.drive.setSpeed(speed, speed);
    return false;
  }
}