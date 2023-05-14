package frc.robot.subsystems;

import static frc.robot.Robot.count;
import static frc.robot.Robot.logging;
import static frc.robot.utilities.Util.logf;
import static frc.robot.utilities.Util.round2;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.ctre.phoenix.motorcontrol.LimitSwitchSource;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;

class SRXMotor {
    // TalonSRX motor;
    WPI_TalonSRX motor;
    TalonSRX followMotor;
    String name;
    int id;
    int followId;
    double lastSpeed = 0;
    int lastPos = 0;
    boolean myLogging = false;

    boolean kSensorPhase = false;
    boolean kMotorInvert = false;

    SRXMotor(String name, int id, int followId, boolean logging) {
        this.name = name;
        this.id = id;
        this.followId = followId;
        myLogging = logging;
        // motor = new TalonSRX(id);
        motor = new WPI_TalonSRX(id);
        motor.configFactoryDefault();
        if (followId > 0) {
            followMotor = new TalonSRX(followId);
            followMotor.configFactoryDefault();
            followMotor.follow(motor);
        }
        motor.getSensorCollection().setQuadraturePosition(0, 0);
        if (followId > 0)
            logf("Created %s motor ids:<%d,%d> firmware:<%d,%d> voltage:<%.1f,%.1f>\n", name, id, followId,
                    motor.getFirmwareVersion(), followMotor.getFirmwareVersion(), motor.getBusVoltage(),
                    followMotor.getBusVoltage());
        else
            logf("Created %s motor id:%d firmware:%d voltage:%.1f\n", name, id, motor.getFirmwareVersion(),
                    motor.getBusVoltage());

    }

    public void enableLimitSwitches() {
        motor.configForwardLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyOpen);
        motor.configReverseLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyOpen);
    }

    int getPos() {
        return motor.getSensorCollection().getQuadraturePosition();
    }

    public void setCurrentLimit(int stallLimit, int freeLimit) {
        motor.configContinuousCurrentLimit(freeLimit);
        motor.configPeakCurrentLimit(stallLimit);
    }

    void setBrakeMode(boolean mode) {
        motor.setNeutralMode(mode ? NeutralMode.Brake : NeutralMode.Coast);
    }

    void setPos(double position, boolean debug) {
        // if (debug)
        //     logf("!!!! SetPos %s to %.1f\n", name, position);
        motor.set(ControlMode.Position, position);
        // motor.getSensorCollection().setQuadraturePosition((int) position, 0);
    }

    public int getActualSpeed() {
        return motor.getSensorCollection().getQuadratureVelocity();
    }

    public double getSpeed() {
        return lastSpeed;
    }

    public TalonSRX getMotor() {
        return motor;
    }

    void periodic(String name) {
        if (count % 50 == 0 && logging) {
            logPeriodic();
        }
        if (count % 10 == 0)
            updateSmart(name);
    }

    public void logPeriodic() {
        int pos = getPos();
        if (pos != lastPos) {
            lastPos = pos;
            if (myLogging) {
                if (followId > 0) {
                    logf("%s motor sp:%.2f cur:%.2f temp:%.2f vel:%.2f pos:%.0f\n", name, motor.getMotorOutputPercent(),
                            motor.getStatorCurrent(), motor.getTemperature(), motor.getSelectedSensorVelocity(), pos);
                    logf("%s follow sp:%.2f cur:%.2f temp:%.2f vel:%d pos:%d\n", name,
                            followMotor.getMotorOutputPercent(), followMotor.getStatorCurrent(),
                            followMotor.getTemperature(), followMotor.getSelectedSensorVelocity(),
                            followMotor.getSensorCollection().getQuadraturePosition());
                } else {
                    logf("%s motor sp:%.2f cur:%.2f temp:%.2f vel:%d pos:%d\n", name, motor.getMotorOutputPercent(),
                            motor.getStatorCurrent(), motor.getTemperature(), motor.getSelectedSensorVelocity(),
                            motor.getSensorCollection().getQuadraturePosition());
                }
            }
        }

    }

    public void setCurrentLimit(int peakAmps, int continousAmps, int durationMilliseconds) {
        /*
         * Peak Current and Duration must be exceeded before current limit is activated.
         * When activated, current will be limited to Continuous Current. Set Peak
         * Current params to 0 if desired behavior is to immediately current-limit.
         */
        // talon.configPeakCurrentLimit(35, 10); /* 35 A */
        // talon.configPeakCurrentDuration(200, 10); /* 200ms */
        // talon.configContinuousCurrentLimit(30, 10); /* 30
        motor.configPeakCurrentLimit(peakAmps, Robot.config.kTimeoutMs);
        motor.configPeakCurrentDuration(durationMilliseconds, Robot.config.kTimeoutMs);
        motor.configContinuousCurrentLimit(continousAmps, Robot.config.kTimeoutMs);
        motor.enableCurrentLimit(true); // Honor initial setting
    }

    public void updateSmart(String s) {
        SmartDashboard.putNumber(s + " Pos", (int) getPos());
        SmartDashboard.putNumber(s + " Cur", round2(motor.getStatorCurrent()));
    }

    void setSpeed(double speed) {
        if (speed != lastSpeed) {
            motor.set(ControlMode.PercentOutput, speed);
            // if (followMotor != null) {
            // logf("Set Follow %s speed:%f\n", name, speed );
            // followMotor.set(ControlMode.PercentOutput, speed);
            // }
            lastSpeed = speed;
        }
    }

    void zeroEncoder() {
        motor.getSensorCollection().setQuadraturePosition(0, Robot.config.kTimeoutMs);
    }

    void setEncoderPosition(double position) {
        motor.getSensorCollection().setQuadraturePosition((int) position, Robot.config.kTimeoutMs);
    }

    void setPeakOutput(double output) {
        logf("********************** Set Peak Ouput to: %.1f\n", output);
        motor.configPeakOutputForward(output, Robot.config.kTimeoutMs);
        motor.configPeakOutputReverse(-output, Robot.config.kTimeoutMs);
    }

    void setPositionPID(PID pid, boolean sensorPhase) {
        setPositionPID(motor, 0, sensorPhase);
        PIDToSRX(motor, pid, 0, Robot.config.kTimeoutMs);
    }

    void setVelocityPID(PID pid) {
        // PIDToSRX(motor, pid, 1, Robot.config.kTimeoutMs);
    }

    public void PIDToSRX(TalonSRX srx, PID pid, int slot, int timeout) {
        srx.config_kP(slot, pid.kP, timeout);
        srx.config_kI(slot, pid.kI, timeout);
        srx.config_kD(slot, pid.kD, timeout);
        srx.config_kF(slot, pid.kFF, timeout);
        srx.config_IntegralZone(slot, (int) pid.kIz, timeout);
        srx.configAllowableClosedloopError(slot, pid.allowableCloseLoopError, timeout);
        srx.configMaxIntegralAccumulator(slot, pid.maxIntegralAccumulation, timeout);
        logf("Setup %s PID for %s slot %d %s\n", pid.name, name, slot, pid.getPidData());
    }

    public double getError() {
        return motor.getClosedLoopError(0);
    }

    public void logMotorVCS() {
        if (Math.abs(lastSpeed) > .02) {
            logf("%s\n", getMotorsVCSifRunning(motor));
            if (followId > 0) {
                logf("%s\n", getMotorsVCSifRunning(followMotor));
            }
        }
    }

    public String getMotorVCS() {
        double bussVoltage = motor.getBusVoltage();
        double outputVoltage = motor.getMotorOutputVoltage();
        double supplyCurrent = motor.getSupplyCurrent();
        double statorCurrent = motor.getStatorCurrent();
        return String.format("%s motor volts<%.2f:%.2f> cur<%.2f:%.2f> power<%.2f:%.2f> sp:%.3f", name, bussVoltage,
                outputVoltage, supplyCurrent, statorCurrent, bussVoltage * supplyCurrent, outputVoltage * statorCurrent,
                lastSpeed);
    }

    public String getMotorsVCSifRunning(TalonSRX motor) {
        if (Math.abs(lastSpeed) > .02) {
            double bussVoltage = motor.getBusVoltage();
            double outputVoltage = motor.getMotorOutputVoltage();
            double supplyCurrent = motor.getSupplyCurrent();
            double statorCurrent = motor.getStatorCurrent();
            return String.format("%s motor volts<%.2f:%.2f> cur<%.2f:%.2f> power<%.2f:%.2f> sp:%.3f", name, bussVoltage,
                    outputVoltage, supplyCurrent, statorCurrent, bussVoltage * supplyCurrent,
                    outputVoltage * statorCurrent, lastSpeed);
        }
        return name + "Not Running";
    }

    public double getMotorCurrent() {
        return motor.getStatorCurrent();
    }

    private void setPositionPID(TalonSRX talon, int pidIdx, boolean sensorPhase) {
        logf("!!!!!!! SetPositionPID for %s idx:%d phase:%b invert:%b\n", name, pidIdx, sensorPhase, kMotorInvert);
        // Config the sensor used for Primary PID and sensor direction
        talon.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, pidIdx, Robot.config.kTimeoutMs);

        // Ensure sensor is positive when output is positive
        talon.setSensorPhase(sensorPhase);

        // Set based on what direction you want forward/positive to be.
        // This does not affect sensor phase.
        talon.setInverted(kMotorInvert);

        /* Config the peak and nominal outputs, 12V means full */
        talon.configNominalOutputForward(0, Robot.config.kTimeoutMs);
        talon.configNominalOutputReverse(0, Robot.config.kTimeoutMs);
        talon.configPeakOutputForward(1, Robot.config.kTimeoutMs);
        talon.configPeakOutputReverse(-1, Robot.config.kTimeoutMs);

        // Config the allowable closed-loop error, Closed-Loop output will be neutral
        // within this range. See Table in Section 17.2.1 for native units per rotation.
        talon.configAllowableClosedloopError(0, pidIdx, Robot.config.kTimeoutMs);
    }

    public boolean getForwardLimit() {
        return motor.getSensorCollection().isFwdLimitSwitchClosed();
    }

    public boolean getReverseLimit() {
        return motor.getSensorCollection().isRevLimitSwitchClosed();
    }

}