/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.Relay;
import frc.robot.utilities.Util;

public class RelayController extends SubsystemBase {

  private Relay relay = new Relay(1);

  long count = 0;

  @Override
  public void periodic() {
    if (count == 1)
      resetRelay();
    count--;
  }

  public void toggleRelay() { // Toggle hatch release
    count = 20;
    relay.setDirection(Relay.Direction.kForward);
    setRelay();
  }

  public void setRelay() {
    relay.set(Relay.Value.kOn);
    Util.logf("Operate Relay 1\n");
  }

  public void resetRelay() {
    relay.set(Relay.Value.kOff);
    Util.logf("Release Relay 1\n");
  }
}
