/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import static frc.robot.utilities.Util.round2;
import static frc.robot.utilities.Util.logf;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;

/**
 * An example subsystem. You can replace me with your own Subsystem.
 */
public class DistanceSensors extends SubsystemBase {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  private AnalogInput ultraRear;
  private AnalogInput ultraFront;
  private AnalogInput ballCollectReady;
  private AnalogInput ballShootReady;

  private double backVoltage;
  private double frontVoltage;
  private double ballCollectVoltage;
  private double ballShootVoltage;

  private boolean lastBallShootPressed = false;
  private boolean lastBallCollectPressed = false;
  private double onThresholdCollect = 1.5 + .7;
  private double onThresholdShoot = 1.1; // Was .9 for Monday 3/9 test
  private double offThreshold = .8;

  public DistanceSensors() {
    ultraFront = new AnalogInput(0);
    ultraRear = new AnalogInput(1);
    ballCollectReady = new AnalogInput(2);
    ballShootReady = new AnalogInput(3);
  }


  @Override
  public void periodic() {
    //continuously updates config values for front an rear dist with sensor values
    Robot.frontDist = getFrontInches();
    Robot.rearDist = getRearInches();
    if (Robot.count % 16 == 1)
      updateSmartDashboard();
  }

  public double getRearInches() {
    backVoltage = ultraRear.getVoltage();
    if (backVoltage < .2)
      return 0;
    return round2(backVoltage * 44);

  }

  public double getBallCollect() {
    //reads the voltage from the intake sensor and returns if it is active
    //(active when ball present, inactive when not sensing a ball)
    ballCollectVoltage = ballCollectReady.getVoltage();
    if (ballCollectVoltage < .2)
      return 0;
    return round2(ballCollectVoltage);

  }

  public double getBallShoot() {
    //reads voltage from the shooter sensor, returns activity
    //(active when ball present, inactive when not sensing a ball)
    ballShootVoltage = ballShootReady.getVoltage();
    if (ballShootVoltage < .2)
      return 0;
    return round2(ballShootVoltage);

  }

  public boolean rearValid() {
    return ultraRear.getVoltage() > .25;
  }

  public double getFrontInches() {
    frontVoltage = ultraFront.getVoltage();
    if (frontVoltage < .2)
      return 0;
    return round2(frontVoltage * 44);
  }

  public boolean frontValid() {
    return ultraFront.getVoltage() > .25;
  }

  void updateSmartDashboard() {
    SmartDashboard.putNumber("Front In", getFrontInches());
    SmartDashboard.putNumber("Back In", getRearInches());
    SmartDashboard.putNumber("Ball Collect", getBallCollect());
    SmartDashboard.putNumber("Ball Shoot", getBallShoot());
  }

  boolean ballInShootPositionPressed() { //for telling conveyor to push ball into shooter
    double voltage = getBallShoot();
    //if the voltage of the shooter sensor is above the threshold for a ball being present
    //and was last inactive, return true
    if (voltage > onThresholdShoot && !lastBallShootPressed) {
      lastBallShootPressed = true;
      logf("+++++++++++++++++ Ball Shoot pressed voltage:%.2f  ++++++++++++++\n", voltage);
      return true;
    }
    //if was last active and voltage of shooter sensor is below threshold for ball present,
    //return false
    if (voltage < offThreshold && lastBallShootPressed) {
      lastBallShootPressed = false;
    }
    return false;
  }

  boolean ballCollectPressed() { //for telling conveyor to push ball from intake into conveyor
    double voltage = getBallCollect();
    //if the voltage of the intake sensor is above the threshold for a power cell present,
    //and wasn't last active, return true
    if (voltage > onThresholdCollect && !lastBallCollectPressed) {
      lastBallCollectPressed = true;
      logf("+++++++ Ball Collect pressed voltage:%.2f ++++++\n", voltage);
      return true;
    }
    //if was last active and the voltage of intake sensor drops below threshold, return false
    if (voltage < offThreshold && lastBallCollectPressed) {
      lastBallCollectPressed = false;
    }
    return false;
  }

  //manual reset to say ball not currently at intake
  //used if intake two balls in a row with no gap for sensor to read
  //that it should move the conveyor again
  void resetBallCollectFlag() {
    lastBallCollectPressed = false;
    logf("+++++++ Ball Collect Flag Reset +++++++\n");
  }
}
