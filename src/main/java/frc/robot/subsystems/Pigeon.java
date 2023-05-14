/**
 * Phoenix Software License Agreement
 *
 * Copyright (C) Cross The Road Electronics.  All rights
 * reserved.
 * 
 * Cross The Road Electronics (CTRE) licenses to you the right to 
 * use, publish, and distribute copies of CRF (Cross The Road) firmware files (*.crf) and 
 * Phoenix Software API Libraries ONLY when in use with CTR Electronics hardware products
 * as well as the FRC roboRIO when in use in FRC Competition.
 * 
 * THE SOFTWARE AND DOCUMENTATION ARE PROVIDED "AS IS" WITHOUT
 * WARRANTY OF ANY KIND, EITHER EXPRESS OR IMPLIED, INCLUDING WITHOUT
 * LIMITATION, ANY WARRANTY OF MERCHANTABILITY, FITNESS FOR A
 * PARTICULAR PURPOSE, TITLE AND NON-INFRINGEMENT. IN NO EVENT SHALL
 * CROSS THE ROAD ELECTRONICS BE LIABLE FOR ANY INCIDENTAL, SPECIAL, 
 * INDIRECT OR CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST DATA, COST OF
 * PROCUREMENT OF SUBSTITUTE GOODS, TECHNOLOGY OR SERVICES, ANY CLAIMS
 * BY THIRD PARTIES (INCLUDING BUT NOT LIMITED TO ANY DEFENSE
 * THEREOF), ANY CLAIMS FOR INDEMNITY OR CONTRIBUTION, OR OTHER
 * SIMILAR COSTS, WHETHER ASSERTED ON THE BASIS OF CONTRACT, TORT
 * (INCLUDING NEGLIGENCE), BREACH OF WARRANTY, OR OTHERWISE
 */

/**
 * PLEASE READ FIRST
 * 
 * Description:
 * The PigeonRemoteSensor example demonstrates:
 *  - how to get values from a PigeonIMU and
 *  - how to to use a PigeonIMU as a remote sensor
 * Tested with Logitech F350 USB Gamepad inserted into Driver Station
 * 
 * Be sure to configure the correct motor controller and pigeon
 * The example has all the tools needed to determine if the pigeon is in a good
 * position on the robot i.e. it will measure the desired values under all
 * circumstances the robot will experience.
 * 
 * The first thing you should do is determine if the placement of the pigeon is
 * close enough to the center of rotation - The Pigeon IMU is a sensor that is
 * impacted by excessive and sustained g-forces. Keeping it near the center of
 * rotation will prevent this excessive and sustained g-force while the robot is
 * rotating. The procedure for this is as follows: 1. Drive the robot into an
 * immovable flat obstacle, such as a wall 2. Zero the yaw and accumZ by
 * pressing the A button 3. Drive the robot in a zero turn at max speed for
 * about 30 seconds 4. Drive the robot in the opposite direction for about 30
 * seconds or until yaw is around 0 5. Drive back up to the immovable object,
 * and measure the yaw value and accum Z value 6. If yaw is incorrect and accumZ
 * is correct, move the IMU closer to Centor of rotation and repeat
 * 
 * The second thing you should do is temperature-calibrate the pigeon - Follow
 * the guide in the documentation Bring-Up Pigeon for this
 * https://phoenix-documentation.readthedocs.io/en/latest/ch11_BringUpPigeon.html#temperature-calibration
 * 
 * A quick guide on how to temperature calibrate is below: 1. Ensure pigeon is
 * cool before beginning temperature calibration. This can be confirmed with a
 * self test 2. Enter temperature calibration mode. This is done either using
 * the API or using Phoenix Tuner 3. Heat the pigeon. 4. Once the pigeon has
 * seen a sufficient range of temperatures, it will momentarily blink green,
 * then cleanly boot-calibrate.
 * 
 * Controls:
 * X, B Buttons: Cycle between yaw, pitch, and roll as the selected filter for the motor controller
 * A Button: Zero yaw
 * Y Button: Start printing information every 50 loops
 * Left, Right Bumpers: Cycle between printing YPR information, and any other information pigeon has
 * Left joystick up/down: Throttle for the Robot
 * Right joystick left/right: Turn for the Robot
 */

package frc.robot.subsystems;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.sensors.PigeonIMU;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.utilities.Util;

public class Pigeon extends SubsystemBase {

    /** pigeon instance, this is instantiated later, leave this alone */
    PigeonIMU pidgey;

    /**
     * if Pigeon is on CAN-bus, enter the device ID. its connected to Talon, this
     * does not matter
     */
    final int kPigeonID = 10;

    /* timeouts for certain blocking actions */
    final int kTimeoutMs = 50;

    public Pigeon() {
        /* create the pigeon */
        pidgey = new PigeonIMU(kPigeonID);
        Util.logf("Create Pigeon firmware:%d deviceID:%d\n", pidgey.getFirmwareVersion(), pidgey.getDeviceID());
        zeroYaw();
    }

    @Override
    public void periodic() {
        double[] ypr = new double[3];
        ErrorCode error = pidgey.getYawPitchRoll(ypr);
        if (error != null) {
            SmartDashboard.putString("Pigeon Error", error.toString());
        }
        double yaw = Util.round2(Util.normalizeAngle(ypr[0]) * -1);
        SmartDashboard.putNumberArray("PigeonYPR", ypr);
        SmartDashboard.putNumber("Pigeon YAW", yaw);
    }

    public void zeroYaw() {
        pidgey.setYaw(0, kTimeoutMs);
        pidgey.setAccumZAngle(0, kTimeoutMs);
        System.out.println("Yaw and accumulated Z zero'ed for Piegon");
    }

}