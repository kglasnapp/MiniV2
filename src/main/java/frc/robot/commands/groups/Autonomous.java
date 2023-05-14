package frc.robot.commands.groups;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.commands.DisplayLog;
import frc.robot.utilities.Util;

/**
 * An example command. You can replace me with your own command.
 */
public class Autonomous extends CommandBase {

    public enum AutonomousMode {
        RightOfGoal, Center, LeftOfGoal, DelayRight, DelayCenter, DelayLeft, CrossLine, Test
    };

    AutonomousMode mode;
    boolean test = false;

    public Autonomous(AutonomousMode mode) {
        Util.loginfo("Prepare for Ball Collection mode:%s angle:%d\n", mode, Robot.config.collectAngle);
        new DisplayLog("Start Test");
    }
}
