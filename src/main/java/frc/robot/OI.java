/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import static frc.robot.utilities.Util.log;
//import static frc.robot.utilities.Util.logf;

import java.util.ArrayList;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.robot.commands.DriveStraight;
import frc.robot.commands.DriveStraight.DriveMode;
import frc.robot.commands.MoveBallConveyor;
import frc.robot.commands.PointTurretCommand;
import frc.robot.commands.ShootAllBalls;
import frc.robot.commands.TestBallShooter;
import frc.robot.commands.TestFunctions;
import frc.robot.commands.TurnTo;
import frc.robot.commands.TurnTo.TurnMode;
import frc.robot.commands.TurretCommand;
import frc.robot.commands.ZeroEncoders;
import frc.robot.commands.ZeroYaw;
import frc.robot.subsystems.Joysticks;

/**
 * This class is the glue that binds the controls on the physical operator
 * interface to the commands and command groups that allow control of the robot.
 */

// Shooter Buttons
// operstor 1 -- Button A -- Test Shooter motor
// Operator 2 -- Button B -- Turn on ball conveyor
// Operator 4 -- Button Y -- Point Turret based upon limielight
// Operator 5 -- Left Button -- Drive Straight
// Operator 6 -- Right Button -- Shoot Balls
// Operator 9 -- Press Left Jot -- Zero Yaw
// Operator POV -- UP -- Center Turret
// Operator POV -- Right -- Move Turret to the right
// Operator POV -- Left -- Move Turret to the left
// Operator POV -- Down -- Home the Turret
// Operator JOY Left -- Turn to the left
// Operator JOY Right -- Turn to the right

public class OI {

  static enum Action {
    WhenPressed, WhenReleased;
  };

  private ArrayList<ButtonHandler> buttons = new ArrayList<ButtonHandler>();
  Joystick op = Robot.joysticks.getOperatorJoy();

  OI() {

    new ButtonHandler(op, 9, Action.WhenPressed, new ZeroYaw(), "Zero the Yaw");
    new ButtonHandler(op, 6, Action.WhenPressed, new ShootAllBalls(true), "Shoot All Balls");
    POVButton POVOPLeft = new POVButton(op, 270);
    POVOPLeft.onTrue(new TurretCommand(285));
    POVButton POVOPRight = new POVButton(op, 90);
    POVOPRight.onTrue(new TurretCommand(75));
    POVButton POVOPUp = new POVButton(op, 0);
    POVOPUp.onTrue(new TurretCommand(0));
    POVButton POVOPDown = new POVButton(op, 180);
    POVOPDown.onTrue(new TurretCommand(-1)); // Test turret range of motion and move to 0

    if (Robot.config.limeLightTurret) {
      // op 4 is the Y buttom on the game PAD
      new ButtonHandler(op, 4, Action.WhenPressed, new PointTurretCommand(), "Point Turret ON");
    }
    new ButtonHandler(op, 1, Action.WhenPressed, new TestBallShooter(), "Shoot Command");
    new ButtonHandler(op, 2, Action.WhenPressed, new MoveBallConveyor(-1.0), "Move Ball Conveyor Forward");
    setupSmartDashBoard();
    log("Human Interfaces Created");
  }

  void setupSmartDashBoard() {
    // Smart Dashboard buttons primarly for testing
    SmartDashboard.putData("Turn Left 45", new TurnTo(TurnMode.ABSOLUTE, -45));
    SmartDashboard.putData("Turn Right 45", new TurnTo(TurnMode.ABSOLUTE, 45));
    SmartDashboard.putData("Forward 3 Fast", new DriveStraight(DriveMode.RELATIVE_INCHES, 36, .3, 5));
    SmartDashboard.putData("Back 3 Fast", new DriveStraight(DriveMode.RELATIVE_INCHES, -36, .3, 5));
    SmartDashboard.putData("Forward 1 Foot", new DriveStraight(DriveMode.RELATIVE_INCHES, 12, .3, 5));
    SmartDashboard.putData("Back 1 Foot", new DriveStraight(DriveMode.RELATIVE_INCHES, -12, .3, 5));
    SmartDashboard.putData("ZeroYaw", new ZeroYaw());
    SmartDashboard.putData("ZeroEncoders", new ZeroEncoders());
    SmartDashboard.putData("Test ", new TestFunctions(1));
    SmartDashboard.putData("Test ", new TestFunctions(2));
    SmartDashboard.putData("Test ", new TestFunctions(3));

  }

  public boolean driveStraightPressed() {
    if (Joysticks.leftJoy == null)
      return Joysticks.operator.getRawButtonPressed(5);
    return Joysticks.leftJoy.getTriggerPressed();
  }

  public boolean driveStraightReleased() {
    if (Joysticks.leftJoy == null)
      return Joysticks.operator.getRawButtonReleased(5);
    return Joysticks.leftJoy.getTriggerReleased();
  }

  public double driveStraightSpeed() {
    if (Joysticks.leftJoy == null)
      return 0;
    return (Joysticks.rightJoy.getY() + Joysticks.leftJoy.getY()) / 2;
  }

  public double rightJoySpeed() {
    if (Joysticks.leftJoy == null)
      return 0;
    return Joysticks.rightJoy.getY();
  }

  public double leftJoySpeed() {
    if (Joysticks.leftJoy == null)
      return 0;
    return Joysticks.leftJoy.getY();
  }

  public boolean reverseIntakeWheels() { // Test only
    if (Joysticks.leftJoy == null)
      return false;
    return Joysticks.leftJoy.getRawButton(2);
  }

  public boolean moveBallConveyorButton() { // Test only
    if (Joysticks.leftJoy == null)
      return Joysticks.operator.getRawButton(2);
    return Joysticks.leftJoy.getRawButton(2);
  }

  public double getBottomShooterSpeed() {
    if (Joysticks.leftJoy == null)
      return 0;
    return Joysticks.leftJoy.getThrottle();
  }

  public boolean setDriverCameraMode() {
    if (Joysticks.leftJoy == null)
      return false;
    return Joysticks.leftJoy.getRawButton(10);
  }

  //
  // Right Joy actions
  //

  public boolean getSeekLeft() {
    if (Joysticks.rightJoy == null)
      return false;
    return Joysticks.rightJoy.getPOV() == 270;
  }

  public boolean getSeekRight() {
    if (Joysticks.rightJoy == null)
      return false;
    return Joysticks.rightJoy.getPOV() == 90;
  }

  public boolean shootBalls() {
    if (Joysticks.rightJoy == null)
      return false;
    return Joysticks.rightJoy.getTrigger();
  }

  public boolean getLimelightSeekLeft() {
    if (Joysticks.rightJoy == null)
      return false;
    return Joysticks.rightJoy.getPOV() == 270;
  }

  public boolean getLimelightSeekRight() {
    if (Joysticks.rightJoy == null)
      return false;
    return Joysticks.rightJoy.getPOV() == 90;
  }

  public boolean getRunwayLeft() {
    if (Joysticks.rightJoy == null)
      return false;
    return Joysticks.rightJoy.getRawButton(5);
  }

  public boolean getRunwayRight() {
    if (Joysticks.rightJoy == null)
      return false;
    return Joysticks.rightJoy.getRawButton(6);
  }

  public boolean getLimelightTracking() {
    if (Joysticks.rightJoy == null)
      return false;
    return Joysticks.rightJoy.getPOV() == 0;
  }

  public boolean isOnShootCmd() {
    if (Joysticks.rightJoy == null)
      return false;
    return Joysticks.rightJoy.getTrigger();
  }

  public double getTopShooterSpeed() {
    if (Joysticks.rightJoy == null)
      return 0;
    return Joysticks.rightJoy.getThrottle();
  }

  //
  // Operator actions
  //

  public boolean opSeekTargetTurret() {
    return Joysticks.operator.getRawButton(3);
  }

  public boolean opShootBalls() {
    return Joysticks.operator.getRawButton(5) || Joysticks.operator.getRawButton(6);
  }

  public boolean noTargeting() {
    //logf("Right Trigger %f\n", Joysticks.operator.getRawAxis(3));
    return Joysticks.operator.getRawAxis(3) > .5;
  }

  // Disabled functions
  public boolean turn180() {
    return false;
  }

  public boolean turnToPIDBased() {
    return false;
  }

  public int getPOV() {
    // return Joysticks.operator.getPOV();
    return -1; // Disable this function
  }

  public boolean toggleDetailedLogging() {
    return false;
    // return Joysticks.operator.getRawButtonPressed(8);
  }

  public boolean testShooterWheels() {
    return Joysticks.operator.getRawButton(1);
  }

  class ButtonHandler {
    int port;
    Joystick joystick;
    int buttonNumber;
    Action act;
    String name;

    private ButtonHandler(Joystick joystick, int buttonNumber, Action act, CommandBase cmd, String name) {
      this.joystick = joystick;
      this.buttonNumber = buttonNumber;
      this.act = act;
      this.name = name;
      port = joystick.getPort();
      buttons.add(this);
      JoystickButton button = new JoystickButton(joystick, buttonNumber);
      if (act == Action.WhenPressed)
        button.onTrue(cmd);
      if (act == Action.WhenReleased)
        button.onTrue(cmd);
      // todo took out button.close();
    }

    String getData() {
      return "Button:" + name + " Port:" + port + " Button:" + buttonNumber + " Action:" + act;
    }
  }
}
