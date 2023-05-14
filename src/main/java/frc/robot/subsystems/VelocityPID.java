package frc.robot.subsystems;

import static frc.robot.utilities.Util.logf;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


public class VelocityPID extends SubsystemBase {

  private double kP = .00018;
  private double kI = 1e-6;
  private double kD = 0;
  private double kIz = 0;
  private double kFF = 0;
  private double kMaxOutput = 1;
  private double kMinOutput = -1;
  private String name = "";
  //private Motor motor;
 

  // Set PID using the default PID and maxRPM
  public VelocityPID(String name, Motor motor) {
    this.name = name;
    //this.motor = motor;
    //setVelocityPID();
    // setVelocityPID(motorB);
  }

  // Set PID using the unique values for P, I, D, and maxRPM the remaining values
  // are default
  public VelocityPID(String name, Motor motor, double kP, double kI, double kD, double maxRPM, boolean smartDash) {
    this.name = name;
    this.kP = kP;
    this.kI = kI;
    this.kD = kD;
    this.name = name;
    //this.motor = motor;
    //this.smartDash = smartDash;
    //setVelocityPID();
    // this.maxRPM = maxRPM;
    // setVelocityPID(motorA);
    // setVelocityPID(motorB);
  }

 

  void putToSmart() {
    // display PID coefficients on SmartDashboard
    SmartDashboard.putNumber("P Gain", kP);
    SmartDashboard.putNumber("I Gain", kI);
    SmartDashboard.putNumber("D Gain", kD);
    SmartDashboard.putNumber("I Zone", kIz);
    SmartDashboard.putNumber("Feed Forward", kFF);
    SmartDashboard.putNumber("Max Output", kMaxOutput);
    SmartDashboard.putNumber("Min Output", kMinOutput);
  }

  // @Override
  // public void periodic() {
  // }

  public boolean checkForPIDChanges() {
    // read PID coefficients from SmartDashboard
    double p = SmartDashboard.getNumber("P Gain", 0);
    double i = SmartDashboard.getNumber("I Gain", 0);
    double d = SmartDashboard.getNumber("D Gain", 0);
    double iz = SmartDashboard.getNumber("I Zone", 0);
    double ff = SmartDashboard.getNumber("Feed Forward", 0);
    double max = SmartDashboard.getNumber("Max Output", 0);
    double min = SmartDashboard.getNumber("Min Output", 0);

    // if PID coefficients on SmartDashboard have changed
    boolean change = false;
    if ((p != kP)) {
      change = true;
      logf("PID old p:%f new p:%f\n", kP, p);
      kP = p;
    }
    if ((i != kI)) {
      change = true;
      logf("PID old i:%f new i:%f\n", kP, p);
      kI = i;
    }
    if ((d != kD)) {
      change = true;
      logf("PID old d:%f new d:%f\n", kP, p);
      kD = d;
    }
    if ((iz != kIz)) {
      change = true;
      kIz = iz;
    }
    if ((ff != kFF)) {
      change = true;
      kFF = ff;
    }
    if ((max != kMaxOutput) || (min != kMinOutput)) {
      change = true;
      kMinOutput = min;
      kMaxOutput = max;
    }
    if (change) {
      logf("Pid Changed new:%s\n", getPIDData());
    }
    return change;
  }

  public String getPIDData() {
    return String.format("Name:%s P:%f I:%f D:%f IZ:%f FF:%f Min:%f Max:%f", name, kP, kI, kD, kIz, kFF, kMinOutput, kMaxOutput);
  }

}
