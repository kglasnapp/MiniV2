package frc.robot.subsystems;

import static frc.robot.utilities.Util.logf;
import static frc.robot.utilities.Util.normalizeAngle;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.subsystems.Motor.MotorTypes;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

// A subsytem to control the shooter motor
public class TurretShooter extends SubsystemBase {
  private Motor ballShootBottom;
  private Motor ballIntakeHorizontal;
  private Motor ballIntakeVertical;
  private Motor ballIntakeConveyor;

  private Motor turret;
  public VelocityPID velocityPIDBottom;
  public PID turretPositionPID;
  // Global Speeds are set by commands that set the speed
  private Double globalBottomSpeed = 0.000001;
  // Prepared speeds come from the set PreparsShooter.java
  private double preparedShootSpeedBottom = 0;
  private double preparedShootDistance = 0;

  private double maxRPM = Robot.config.revRPM;
  private double setPointBottom = 0;
  private double lastSetpointBottom = 0;

  private double TurretTicksPerDegree = 7812;
  private double turretMaxAngle = 85;
  private double turretSetPoint = 0;
  private double turretAllowableError = 0.5;
  private double leftLimitEncoderPosition = 694054;

  public AimTurret aim = new AimTurret();
  // private boolean targetFound = true;
  // private boolean moveTurret = true;
  private boolean lastButton = false;

  // Put methods for controlling the shooter subsystem here.
  // The methods in this subsystem should be called Commands.

  // Called when the subsystem is started
  public TurretShooter() {
    if (Robot.config.bottomShooterID < 0) {
      logf("Shooter not created\n");
      return;
    }

    ballShootBottom = new Motor(MotorTypes.SRX, "Ball Shooter Bottom", Robot.config.bottomShooterID, -1, true);
    // speedControlBottom = new SpeedControl(0.08, 0.05);
    velocityPIDBottom = new VelocityPID("Ball Shooter", ballShootBottom, 0.00018, 0, 0, 5800, false);
    ballIntakeHorizontal = new Motor(MotorTypes.SRX, "In Horz", Robot.config.intakeHorizontalID, -1, true);
    ballIntakeVertical = new Motor(MotorTypes.SRX, "In Vert", Robot.config.intakeVerticalID, -1, true);
    ballIntakeConveyor = new Motor(MotorTypes.SRX, "In Lower", Robot.config.intakeConveryorID, -1, true);
    turret = new Motor(MotorTypes.SRX, "Turret", Robot.config.turretID, -1, true);
    turretPositionPID = new PID("position", .1, .0, 6, 0.0, 0.0, 0.0, 0.0, 0, false);
    logf("**** Setup Postion PID for Turret -> %s\n", turretPositionPID.getPidData());
    turret.setPositionPID(turretPositionPID, true);
    turret.enableLimitSwitches();
    turret.zeroEncoder();
    turret.getSRXMotor().setCurrentLimit(0, 1, 10);
    // turret.setInverted(true);

    int limit = 40;
    ballShootBottom.getSRXMotor().setCurrentLimit(limit, limit);

    if (Robot.config.shooterVelocityPID) {
      logf("++++ Start shooter velocity PID:%s\n", velocityPIDBottom.getPIDData());
    } else {
      logf("++++ Start shooter power based\n");
    }
    ballShootBottom.setBrakeMode(true);
  }

  public void periodic() {
    // Display Turret motor data
    turret.periodic("Turret");
    if (Robot.oi.opSeekTargetTurret()) {
      aim.aimPeriodic();
      lastButton = true;
    } else {
      if (lastButton)
        aim.aimStop();
      lastButton = false;
    }
  }

  public void setShootSpeed(double bottom) {
    bottom = -bottom;
    if (Robot.config.shooterVelocityPID) {
      // set speeds for velocity based pid
      double adjustment = 1.05;
      setPointBottom = bottom * maxRPM * adjustment;
      if (setPointBottom != lastSetpointBottom) {
        // velocityPIDBottom.getPID().setReference(setPointBottom * factor,
        // ControlType.kVelocity);
        logf("New Velocity Setpoint for bot setPoint %.4f\n", setPointBottom);
        lastSetpointBottom = setPointBottom;
      }

    } else {
      // bottom = speedControlBottom.setSpeed(bottom);
      // todo stop commpressor during shooting
      lastSetpointBottom = bottom;
      ballShootBottom.setSpeed(bottom);
    }

  }

  public void setBallConveyorSpeed(double speed) {
    ballIntakeConveyor.set(speed);
  }

  public void setShootSpeedActual(double speed) {
    ballShootBottom.setSpeed(speed);
  }

  public void ballIntakeOff() {
    ballIntakeHorizontal.set(0);
    ballIntakeVertical.set(0);
    ballIntakeConveyor.set(0);
  }

  public void ballIntakeOn() {
    ballIntakeHorizontal.set(-1.0);
    ballIntakeVertical.set(1.0);
    ballIntakeConveyor.set(-1.0);
  }

  public void turretSetAngle(double angle, boolean debug) {
    angle = normalizeAngle(angle);
    if (Math.abs(angle) > turretMaxAngle) {
      logf("Error: Tried to set turret angle out of range %f\n", angle);
      return;
    }
    turretSetPoint = -angle;
    turret.setActualPosition(angle * TurretTicksPerDegree, debug);
  }

  public void turretSetSpeed(double speed) {
    turret.setSpeed(speed);
  }

  public boolean isTurretFinished() {
    double err = Math.abs(getTurretAngle() - turretSetPoint);
    // logf("isTurretFinihsed err:%.1f\n", err);
    return (err < turretAllowableError);
  }

  public void turretMoveToLimit(double angle) {
    angle = normalizeAngle(angle);
    double position = angle * TurretTicksPerDegree;
    turretSetPoint = -angle;
    turret.setActualPosition(position, true);
  }

  public void setTurretPositionAtLeftLimit() {
    turret.getSRXMotor().setEncoderPosition(leftLimitEncoderPosition);
  }

  public String getTurretVCS() {
    return turret.getSRXMotor().getMotorVCS();
  }

  public int getTurretPosition() {
    return turret.getEncoder();
  }

  public double getTurretAngle() {
    return turret.getEncoder() / TurretTicksPerDegree;
  }

  public boolean leftLimitTurret() {
    return turret.getSRXMotor().getReverseLimit();
  }

  public boolean rightLimitTurret() {
    return turret.getSRXMotor().getForwardLimit();
  }

  void show() {
    SmartDashboard.putNumber("Bot Speed " + ballShootBottom.name, ballShootBottom.getSpeed());
  }

  public double getBottomActualSpeed() {
    return ballShootBottom.getSRXMotor().getActualSpeed();
  };

  public double getBottomPositon() {
    return ballShootBottom.getEncoder();
  }

  public double getBottomRPM() {
    return ballShootBottom.getRPM();
  }

  public double getGlobalBottomSpeed() {
    return globalBottomSpeed;
  }

  public double getLastSetpointBottom() {
    return lastSetpointBottom;
  }

  public void setGlobalBottomSpeed(double speed) {
    globalBottomSpeed = speed;
  }

  public String getPIDDataBottom() {
    return velocityPIDBottom.getPIDData();
  }

  public void setPreparedShootSpeeds(double top, double bottom, double distance) {
    preparedShootSpeedBottom = bottom;
    preparedShootDistance = distance;
  }

  public double getPreparedBottomSpeed() {
    return preparedShootSpeedBottom;
  }

  public double getPreparedDistance() {
    return preparedShootDistance;
  }

  public double getBottomCurrent() {
    return ballShootBottom.getCurrent();
  }

  // Determine if the shooter is active
  public boolean active() {
    double bot = getGlobalBottomSpeed();
    boolean active = Math.abs(bot) > .05;
    logf("!!!!!!!!!!!!!!!!!!! Shooter -- Active:%b  bot:%.1f\n", active, bot);
    return active;
  }

  static enum States {
    Idle, LLValid, Located, TurnToLeft, TurnLeft, TurnToRight, TurnRight
  };

  public class AimTurret {
    private States state = States.Idle;

    AimTurret() {
      state = States.Idle;
      logf("Init aimTurret State %s\n", state);
    }

    public boolean aimPeriodic() {
      boolean limeLightValid = Robot.limeLightVision.valid;
      double angle = Robot.limeLightVision.x;
      double turretAngle = Robot.turretShooter.getTurretAngle();
      logf("Seek turret to %s LLValdid:%b LLAng:%.1f Turret Ang:%.1f leftLim:%b rightLim:%b\n", state, limeLightValid,
          angle, turretAngle, leftLimitTurret(), rightLimitTurret());
      switch (state) {
        case Idle:
          if (limeLightValid) {
            state = States.LLValid;
          } else {
            state = States.TurnToLeft;
          }
          break;
        case LLValid:
          if (Math.abs(angle - turretAngle) < .5) {
            state = States.Located;
            break;
          }
          Robot.turretShooter.turretSetAngle(angle - turretAngle, true);
          break;
        case Located:
          if (limeLightValid) {
            if (Math.abs(angle - turretAngle) > .5) {
              Robot.turretShooter.turretSetAngle(angle - turretAngle, true);
            } else {
              Robot.turretShooter.turretSetSpeed(0);
              state = States.Idle;
              return true;
            }
          } else {
            state = States.TurnToLeft;
          }
          break;
        case TurnToLeft:
          Robot.turretShooter.turretSetAngle(68, true);
          state = States.TurnLeft;
          break;
        case TurnLeft:
          if (turretAngle < -66) {
            state = States.TurnToRight;
          } else if (limeLightValid) {
            state = States.LLValid;
          }
          break;
        case TurnToRight:
          Robot.turretShooter.turretSetAngle(-68, true);
          state = States.TurnRight;
          break;
        case TurnRight:
          if (turretAngle > 66) {
            state = States.TurnToLeft;
          } else if (limeLightValid) {
            state = States.LLValid;
          }
          break;
      }
      return false;
    }

    public void aimStop() {
      logf("Aim Stop State %s\n", state);
      Robot.turretShooter.turretSetSpeed(0);
      state = States.Idle;
    }
  }

}
