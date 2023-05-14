package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.utilities.*;
//import frc.robot.subsystems.SpeedControl;
//import frc.robot.subsystems.Drive;

/**
 * An example command. You can replace me with your own command.
 */
public class TestFunctions extends CommandBase {

    // 1 - 
    // 2 - 
    // 3 - 
    private int testId = 0;

    private int testPtr = 0;

    // test values for 

    public TestFunctions(int testId) {
        this.testId = testId;
        // Use requires() here to declare subsystem dependencies
    }

    // Called just before this Command runs the first time
    @Override
    public void initialize() {
        Util.logf("++++ TestFunctions Command Initialized Testid:%d Pointer:%d\n", testId, testPtr);
        testPtr = -1;
    }

    // Called repeatedly when this Command is scheduled to run
    @Override
    public void execute() {
    }

    // Make this return true when this Command no longer needs to run execute()
    @Override
    public boolean isFinished() {
        testPtr++;
        // Test for 
        if (testId == 1) {
           
            return false;
        }
         // Test for 
        if (testId == 2) {

            return false;
        }

        if (testId == 3) {
            
            return false;
        }
        return true;
    }

    // Called once after isFinished returns true
    @Override
    public void end(boolean interrupted) {
        Util.logf("---- TestFunctions Command ended\n");
    }


}
