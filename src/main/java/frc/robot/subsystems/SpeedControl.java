/*----------------------------------------------------------------------------*/
/* Copyright (c) 2020 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

public class SpeedControl {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.
  Double decrement = 0.0;
  Double increment = 0.0;

  Double lastSpeed = 0.0;
  Double incrementedSpeed = 0.0;

  boolean speedMode; // false if power, true if speed

  double desiredSpeed;
  double currentSpeed;

  public SpeedControl(double decrement, double increment) {
    this.decrement = decrement;
    this.increment = increment;
  }

  // public double getLastSpeed(){
  // return lastSpeed;
  // }

  public double setSpeed(double desiredSpeed) {
    this.desiredSpeed = desiredSpeed;
    if (lastSpeed > desiredSpeed) { // then decrement
      lastSpeed -= decrement;
      if (lastSpeed <= desiredSpeed) {
        lastSpeed = desiredSpeed;
      }
    }
    if (lastSpeed < desiredSpeed) {
      lastSpeed += increment;
      if (lastSpeed >= desiredSpeed) {
        lastSpeed = desiredSpeed;
      }
    }
    this.currentSpeed = lastSpeed;
    return lastSpeed;
  }

  // public boolean isAtDesired() {
  // Util.logf("Current speed %.2f, desired speed %.2f\n", currentSpeed,
  // desiredSpeed);
  // if (currentSpeed == desiredSpeed) {
  // return true;
  // }
  // else {
  // return false;
  // }
  // }

  public double setSpeedTinyBug(double desiredSpeed) {
    // Util.logf("setting speed desired %.1f, last %.1f, incremented %.1f \n",
    // desiredSpeed, lastSpeed, incrementedSpeed);
    if (lastSpeed > desiredSpeed) // then decrement
    {
      incrementedSpeed -= increment;
      if (incrementedSpeed < desiredSpeed || incrementedSpeed == desiredSpeed) {
        incrementedSpeed = desiredSpeed;
        lastSpeed = desiredSpeed;
      }
    }
    if (lastSpeed < desiredSpeed) { // then increment
      incrementedSpeed += increment;
      if (incrementedSpeed > desiredSpeed || incrementedSpeed == desiredSpeed) {
        incrementedSpeed = desiredSpeed;
        lastSpeed = desiredSpeed;
      }
    }
    return incrementedSpeed;
    // log void method which had motor passed into constructor and set speed to
    // motor
    // from here
    // return desiredSpeed;
  }
}
