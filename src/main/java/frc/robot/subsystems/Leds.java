/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * An example subsystem. You can replace me with your own Subsystem.
 */
public class Leds extends SubsystemBase {
    // Put methods for controlling this subsystem
    // here. Call these from Commands.
    //public static MySolenoid led0 = new MySolenoid(0, 0);
    //public static MySolenoid led1 = new MySolenoid(0, 1);
    //public static MySolenoid led2 = new MySolenoid(0, 2);
    //public static MySolenoid led3 = new MySolenoid(0, 3);
    
    //Orient Shooter Limit Switches
    //public static MySolenoid led4 = new MySolenoid(0, 4); //elevate forward
    //public static MySolenoid led5 = new MySolenoid(0, 6); //elevate reverse //change orders of ids b/c skips five b/c its belowp but not led?
    //public static MySolenoid led6 = new MySolenoid(0, 7); //azimuth foward
    //public static MySolenoid led7 = new MySolenoid(0, 8); //azimuth reverse

    //public static MySolenoid[] leds = new MySolenoid[] { led0, led1, led2, led3, led4, led5, led6, led7 };
    //public static int lastLed = 7; //correct since only 7 leds since skipping five?
    //public MySolenoid limeLightLed = new MySolenoid(0, 5);


    @Override
    public void periodic() {
    }

}
