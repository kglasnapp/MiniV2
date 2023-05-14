package frc.robot.commands.groups;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.commands.DriveStraight;
import frc.robot.commands.DriveStraight.DriveMode;
import frc.robot.commands.Timeout;
import frc.robot.commands.TurnTo;
import frc.robot.commands.TurnTo.TurnMode;
import frc.robot.commands.ZeroYaw;
import frc.robot.utilities.Util;

/**
 * An example command. You can replace me with your own command.
 */
public class DrivePolygon extends CommandBase {
    public DrivePolygon(int sides, double distance, double speed) {
        Util.loginfo("DrivePoly sides:%d dist:%.1f speed:%.1f\n", sides,distance,speed);
        double timeout = .25;
        new ZeroYaw();
        for (int i = 0; i < sides; i++) {
           new Timeout(timeout);
           new DriveStraight(DriveMode.RELATIVE_INCHES, distance, speed, 5);
           new TurnTo(TurnMode.RELATIVE, 360 / sides);
        }
    }
}
