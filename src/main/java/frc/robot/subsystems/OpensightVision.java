/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import static frc.robot.utilities.Util.loginfo;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import static frc.robot.utilities.Util.round2;
import frc.robot.Robot;

/**
 * Class used to use OpenSight with our robot.
 */
public class OpensightVision extends SubsystemBase {
  NetworkTable table = NetworkTableInstance.getDefault().getTable("OpenSight");
  NetworkTableEntry coord_x = table.getEntry("coord-x");
  NetworkTableEntry coord_y = table.getEntry("coord-y");
  NetworkTableEntry success = table.getEntry("succ");
  String lastMessage = "";


  @Override
  public void periodic() {
    if (Robot.count % 50 == 0) { // Check for changes every 1000 MS
      double x = round2(coord_x.getDouble(999));
      double y = round2(coord_y.getDouble(998));
      boolean valid = success.getBoolean(false);
      String message = String.format("OpenSight valid:%b x:%.1f y:%.1f\n", valid, x, y);
      if (!message.equals(lastMessage)) { // Do a log only if the message has changed
        loginfo(message);
        lastMessage = message;
      }
      SmartDashboard.putNumber("OPSI X", x);
      SmartDashboard.putNumber("OPSI Y", y);
      SmartDashboard.putString("OPSI Valid", valid ? "True" : "False");
    }
  }
}