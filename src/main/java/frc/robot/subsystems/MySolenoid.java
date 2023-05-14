/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.utilities.Util;

public class MySolenoid extends SubsystemBase {
    // This class creates solenoid objects for things like LEDs
    // that are controlled by a Pneumatic Control Module (PCM)
    // It will warn you if the PCM does not exist

    Solenoid m_Solenoid;
    int id;
    int channelA;
    // Create a single channel solenoid -- used for controlling things like leds
    public MySolenoid( int id, int channelA) {
        if (!((id == 0 && Robot.config.pcmID0) || (id == 1 && Robot.config.pcmID1))) {
            m_Solenoid = null;
            Util.logf("No solenoid configuration for id:%d channel:%d\n", id, channelA);
            return;
        }
        try {
            m_Solenoid = new Solenoid(PneumaticsModuleType.CTREPCM, channelA);
            this.id = id;
            this.channelA = channelA;
            Util.loginfo("Create a single solenoid for id:%d channel:%d\n", id, channelA);
        } catch (Exception e) {
            m_Solenoid = null;
            Util.logf("*** Error unable to create a single solenoid for id:%d\n", id);
        }
    }

    public boolean get() {
        if (m_Solenoid != null)
            return m_Solenoid.get();
        else
            return false;
    }

    public void set(boolean value) {
        if (m_Solenoid != null)
            m_Solenoid.set(value);
    }
}
