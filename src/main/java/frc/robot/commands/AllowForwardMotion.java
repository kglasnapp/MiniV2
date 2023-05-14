package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
//import frc.robot.Robot;
import frc.robot.utilities.*;

/**
 * An example command. You can replace me with your own command.
 */
public class AllowForwardMotion extends CommandBase {
    public AllowForwardMotion() {
        // Use requires() here to declare subsystem dependencies
    }

    // Called just before this Command runs the first time
    @Override
    public void initialize() {
        Util.logf("++++  AllowForwardMotion Command Initialize\n");
    }

    // Called repeatedly when this Command is scheduled to run
    @Override
    public void execute() {
    }

    // Make this return true when this Command no longer needs to run execute()
    @Override
    public boolean isFinished() {
        return true;
    }

    // Called once after isFinished returns true
    @Override
   public void end(boolean interrupted) {
        Util.logf("---- AllowForwardMotion Command ended\n");
    }

}
