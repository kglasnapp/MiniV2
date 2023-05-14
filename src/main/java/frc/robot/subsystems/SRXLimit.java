package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.ctre.phoenix.motorcontrol.LimitSwitchSource;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import static frc.robot.utilities.Util.logf;
import frc.robot.Robot;
import frc.robot.subsystems.NeoPixelControl.LedAssignment;
import frc.robot.subsystems.NeoPixelControl.LedType;

class SRXLimit {
    private boolean lastLimit = false;
    private LimitType type;
    private String majorName, switchName;
    private LedAssignment neoPixel;
    private Motor motor;

    public enum LimitType {
        FORWARD, REVERSE, CENTER
    };

    SRXLimit(Motor motor, String majorName, String switchName, LimitType type, LedAssignment neoPixel) {
        if (motor.type == Motor.MotorTypes.SRX) {
            if (type == LimitType.FORWARD)
                motor.getSRXMotor().getMotor().configForwardLimitSwitchSource(LimitSwitchSource.FeedbackConnector,
                        LimitSwitchNormal.NormallyOpen);
            if (type == LimitType.REVERSE)
                motor.getSRXMotor().getMotor().configReverseLimitSwitchSource(LimitSwitchSource.FeedbackConnector,
                        LimitSwitchNormal.NormallyOpen);
        }
        // motor.getSRXMotor().getMotor().configForwardLimitSwitchSource(type,
        // normalOpenOrClose, timeoutMs)
        this.majorName = majorName;
        this.switchName = switchName;
        this.type = type;
        this.neoPixel = neoPixel;
        this.motor = motor;
    }

    boolean get() {
        if (type == LimitType.FORWARD)
            return motor.getSRXMotor().getMotor().getSensorCollection().isFwdLimitSwitchClosed();
        if (type == LimitType.REVERSE)
            return motor.getSRXMotor().getMotor().getSensorCollection().isRevLimitSwitchClosed();
        return false;
    }

    // Returns true if limit switch hit
    boolean periodic() {
        if (Robot.count % 10 == 0)
            SmartDashboard.putNumber(String.format("%s %s sw", majorName, switchName), get() ? 1 : 0);
        if (get()) {
            if (!lastLimit) { // limit switch hit
                lastLimit = true;
                logf("%s %s (%s) limit switch hit\n", majorName, switchName, type);
                if (neoPixel != null)
                    Robot.neoPixelControl.setLed(neoPixel, LedType.BLUE);
                return true;
            }
        } else {
            if (neoPixel != null)
                if (lastLimit)
                    Robot.neoPixelControl.setLed(neoPixel, LedType.GREEN);
            lastLimit = false;
        }
        return false;
    }
}