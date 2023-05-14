
package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.utilities.Util;

/**
 * Used in command groups to wait between commands.
 */

public class Timeout extends CommandBase {
    private final double timeoutValue;
    Timer timer = new Timer();

    /**
     * @param timeout The timeout in seconds. Cannot be less than 0.
     * @throws IllegalArgumentException if given a negative timeout
     */
    public Timeout(double timeoutValue) {
        this.timeoutValue = timeoutValue;
    }

    @Override
    public void initialize() {
        Util.logf("Timeout cmd started. Seconds=%.1f\n", timeoutValue);
        timer.start();
    }

    @Override
    public boolean isFinished() {
        return timer.hasElapsed(timeoutValue);
    }

    @Override
    public void end(boolean interrupted) {
        Util.logf("Timeout cmd Completed. Seconds=%.1f\n", timeoutValue);
    }
}