package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.subsystems.MySolenoid;
import frc.robot.utilities.*;

/**
 * An example command. You can replace me with your own command.
 */
public class ToggleCameraLight extends CommandBase {
  private MySolenoid light;

  public ToggleCameraLight() {
    // Use requires() here to declare subsystem dependencies
    if (Robot.config.ledRing) {
      light = new MySolenoid(1, 0);
      light.set(false);
    }
  }

  // Called just before this Command runs the first time
  @Override
  public void initialize() {
    Util.logf("++++ Toggle Vison LEDs\n");
    if (Robot.config.limeLight) {
      Util.logf(".... Toggle Limelight LED\n");
      Robot.limeLightVision.ledToggle();
    }
    if (Robot.config.ledRing) {
      Util.logf(".... Toggle Led Ring\n");
      light.set(!light.get());
    }
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
    Util.logf("---- Toggle Camera Light ended\n");
  }

}
