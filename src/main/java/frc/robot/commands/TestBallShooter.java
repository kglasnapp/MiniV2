package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;

import static frc.robot.utilities.Util.log;
import static frc.robot.utilities.Util.logf;
import static frc.robot.utilities.Util.round2;

/**
 * An example command. You can replace me with your own command.
 */
public class TestBallShooter extends CommandBase {
    private String lastMessage = "";

    public TestBallShooter() {
        // Use requires() here to declare subsystem dependencies
        //requires(Robot.turretShooter);
    }

    // Called just before this Command runs the first time
    @Override
    public void initialize() {
        if (Robot.config.bottomShooterID > 0) {
            double bottomSpeed = Robot.turretShooter.getGlobalBottomSpeed();
            logf("Start Ball Shooter with speed  bot:%f pid:%s\n", bottomSpeed, Robot.turretShooter.getPIDDataBottom());

        }
    }

    // Called repeatedly when this Command is scheduled to run
    @Override
    public void execute() {
    }

    // Make this return true when this Command no longer needs to run execute()
    @Override
    public boolean isFinished() {
        double bottomSpeed = 0;
        if (Robot.config.bottomShooterID < 0)
            return true;
        // todo fix this
        if (Robot.oi.testShooterWheels()) {
            bottomSpeed = 0.5;
            Robot.turretShooter.setShootSpeed(bottomSpeed);
        } else {
            bottomSpeed = 0;
            Robot.turretShooter.setShootSpeed(bottomSpeed);
        }
        if (Robot.count % 20 == 0) { // Make a log every 200 milliseconds only if the data changed
            double bottomActualSpeed = Robot.turretShooter.getBottomActualSpeed();
            double bottomCurrent = Robot.turretShooter.getBottomCurrent();
            String message = String.format("Shooter setPoint: b:%.3f desired b:%.3f RPM  b:%.1f Current  b:%.1f",
                    Robot.turretShooter.getLastSetpointBottom(), bottomSpeed, bottomActualSpeed, bottomCurrent);
            if (!message.equals(lastMessage)) { // Do a log if the message has changed
                log(message);
                lastMessage = message;
                SmartDashboard.putNumber("Bot Shoot Speed", round2(bottomActualSpeed));
            }
        }
        return false;
    }

    // Called once after isFinished returns true
    @Override
    public void end(boolean interrupted) {
        logf("---- Ball Shooter Command ended\n");
    }

    
}