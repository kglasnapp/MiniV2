/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import static frc.robot.utilities.Util.logf;

public class NeoPixelControl extends SubsystemBase {
  private AddressableLED neoPixels;
  private AddressableLEDBuffer ledBuffer;
  // private int lastLight = 0;
  private int numLeds = 32;
  // private int completeCircleCount = 0;
  // private Range blinkRange = new Range(200, 600);

  public enum LedType {
    OFF, RED, GREEN, BLUE, YELLOW, WHITE, PURPLE, TURQUOISE
  };

  public enum LedAssignment {
    Enabled,  Power, RightDriveOK, LeftDriveOK, AllowForward, JoysticksCorrect, OperatorCorrect, FrontDistance,
    RearDistance, Intake, IntakeIn, IntakeOut, BallCollect, BallShoot, Elevate, ElevateForward, ElevateReverse,
    ColorWheelDeploy, ColorWheelRetract, ColorWheelActive, ColorWheelColor,TelscopeSolenoid, LimeLight
  }

  private LedType[] ledType = new LedType[numLeds];

  public NeoPixelControl() {
    if (!Robot.config.neoPixelsActive) {
      return;
    }
    // Must be a PWM header, not MXP or DIO
    neoPixels = new AddressableLED(9);

    // Default to a length of 60, start empty output
    // Length is expensive to set, so only set it once, then just update data
    ledBuffer = new AddressableLEDBuffer(numLeds);
    neoPixels.setLength(numLeds);

    // Set the data
    neoPixels.setData(ledBuffer);
    neoPixels.start();
    for (int i = 0; i < numLeds; i++) {
      ledType[i] = LedType.OFF;
    }

  }

  public void periodic() {
    if (!Robot.config.neoPixelsActive) {
      return;
    }

    if (Robot.count < 24 * 3)
      blink();
    if (Robot.count == (24 * 3 + 1))
      manageLeds();
    if (Robot.count % 100 == 0) {
      double battery = Robot.batteryVoltage;
      if (battery < 12.0)
        setLed(LedAssignment.Power, LedType.RED);
      else if (battery < 12.3)
        setLed(LedAssignment.Power, LedType.YELLOW);
      else
        setLed(LedAssignment.Power, LedType.GREEN);
    }
  }

  void blink() {
    if (Robot.count % 12 == 0) {
      if (Robot.count % 24 == 0)
        for (int i = 0; i < numLeds; i++)
          ledBuffer.setRGB(i, 0, 0, 0);
      else
        for (int i = 0; i < numLeds; i++)
          ledBuffer.setRGB(i, 75, 0, 0);
      neoPixels.setData(ledBuffer); // Set the LEDs
    }
  }

  private void manageLeds() {
    // For every pixel
    for (int i = 0; i < numLeds; i++) {
      manageLed(i);
    }
    neoPixels.setData(ledBuffer); // Set the LEDs
  }

  private void manageLed(int i) {
    switch (ledType[i]) {
    case OFF:
      ledBuffer.setRGB(i, 0, 0, 0);
      break;
    case RED:
      ledBuffer.setRGB(i, 75, 0, 0);
      break;
    case GREEN:
      ledBuffer.setRGB(i, 0, 75, 0);
      break;
    case BLUE:
      ledBuffer.setRGB(i, 0, 0, 75);
      break;
    case WHITE:
      ledBuffer.setRGB(i, 255, 255, 255);
      break;
    case YELLOW:
      ledBuffer.setRGB(i, 255, 255, 0);
      break;
    case PURPLE:
      ledBuffer.setRGB(i, 255, 0, 255);
      break;
    case TURQUOISE:
      ledBuffer.setRGB(i, 0, 255, 255);
      break;
    }
  }

  public void setLed(LedAssignment name, LedType type) {
    if (!Robot.config.neoPixelsActive) {
      return;
    }
    if (ledType[name.ordinal()] == type)
      return; // No need to spend time updating the LED if no change
    ledType[name.ordinal()] = type;
    logf("Set Neopixel <%s> to %s\n", name.toString(), type.toString());
    manageLeds();
  }
}
