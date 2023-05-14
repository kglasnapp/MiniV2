package frc.robot.subsystems;


public class Motor  {

    public static enum MotorTypes {
         SRX
    }

    public MotorTypes type;
   
    private SRXMotor srxMotor;
    public String name;

    Motor(MotorTypes type, String name, int id, int followId, boolean logging) {
        this.type = type;
        this.name = name;
        if (type == MotorTypes.SRX)
            srxMotor = new SRXMotor(name, id, followId, logging);
    }


    public SRXMotor getSRXMotor() {
        return srxMotor;
    }

    public double getRPM() {
        if (type == MotorTypes.SRX)
            return 9999;
        return 0;
    }

    public double getSpeed() {
        if (type == MotorTypes.SRX)
            return srxMotor.getSpeed();
        return 0;
    }

    void periodic(String name) {
        if (type == MotorTypes.SRX)
            srxMotor.periodic(name);
    }

    public int getEncoder() {
        if (type == MotorTypes.SRX)
            return srxMotor.getPos();
        return 0;
    }

    public double getError() {
        if (type == MotorTypes.SRX)
            return srxMotor.getError();
        return 0;
    }

    void setSpeed(double speed) {
        if (type == MotorTypes.SRX)
            srxMotor.setSpeed(speed);
    }

    void zeroEncoder() {
        if (type == MotorTypes.SRX)
            srxMotor.zeroEncoder();
    }

    // Initializes the value of the Encoder
    int initEncoderValue(double position) {
        if (type == MotorTypes.SRX)
            srxMotor.setEncoderPosition(position);
        return 0;
    }

    // Sets the actual position by spinning the motors
    public void setActualPosition(double position, boolean debug) {
        if (type == MotorTypes.SRX)
            srxMotor.setPos(position, debug);
    }

    void setBrakeMode(boolean mode) {
        if (type == MotorTypes.SRX)
            srxMotor.setBrakeMode(mode);
    }

    void enableLimitSwitches() {
        if (type == MotorTypes.SRX)
            srxMotor.enableLimitSwitches();
    }

    // Setup a position PID uses slot 0
    void setPositionPID(PID pid, boolean sensorPhase) {
        if (type == MotorTypes.SRX)
            srxMotor.setPositionPID(pid, sensorPhase);
    }

    // Setup for a velocity PID uses slot 1
    void setVelocityPID(PID pid) {
        if (type == MotorTypes.SRX)
            srxMotor.setVelocityPID(pid);
    }

    public void logMotorVCS() {
        if (type == MotorTypes.SRX)
            srxMotor.logMotorVCS();
    }

    public double getCurrent() {
        if (type == MotorTypes.SRX)
            return srxMotor.getMotorCurrent();
        return 0;
    }

    // Overriding Abstract methods from SpeedController
    public void set(double speed) {
        this.setSpeed(speed);
        return;
    }

    public double get() {
        return this.getSpeed();
    }

    public void pidWrite(double output) {
        // todo
        return;
    }

    public void setInverted(boolean isInverted) {
        // todo
        return;
    }

    public boolean getInverted() {
        // todo
        return false;
    }

    public void disable() {
        // todo
        return;
    }

    public void stopMotor() {
        // todo
        return;
    }
}
