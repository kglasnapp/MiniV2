package frc.robot.subsystems;

import static frc.robot.utilities.Util.log;
import static frc.robot.utilities.Util.logf;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Config;
import frc.robot.Robot;
import frc.robot.Config.DriveType;
import frc.robot.Config.RobotType;
import frc.robot.commands.DriveTeleop;
import frc.robot.subsystems.Motor.MotorTypes;
import edu.wpi.first.wpilibj.drive.MecanumDrive;

public class Drive extends SubsystemBase {

    public Motor rightMotor;
    public Motor leftMotor;

    // For Mecanum Drive: (above motors would be front)
    public Motor rightRearMotor = null;
    public Motor leftRearMotor = null;
    public MecanumDrive mecDrive;

    private String lastMessage = "";

    // private boolean ignoreNext = false;
    private int rightStart = 0;
    private int leftStart = 0;
    private static double maxInput = 1 - Robot.config.deadZone;
    private boolean stopForwardMotion = false;

    public Drive(boolean logging) {
        // configures the drivetrain according to what type of robot and motors are
        // being used (set in config)
        log("Init Drive Subsystem type:" + Robot.config.driveType);
        setDefaultCommand(new DriveTeleop(this));
        if (Config.robotType == Config.RobotType.SuitCase) {
            rightMotor = new Motor(MotorTypes.SRX, "Right Drive", 2, -1, logging);
            leftMotor = new Motor(MotorTypes.SRX, "Left Drive", 3, -1, logging);
        }

        if (Config.robotType == Config.RobotType.SRXMini && !Robot.config.testREVDriveonMini) {
            rightMotor = new Motor(MotorTypes.SRX, "Right Drive", 2, -1, logging);
            leftMotor = new Motor(MotorTypes.SRX, "Left Drive", 3, -1, logging);
        }

        if (Config.robotType == Config.RobotType.SRXMiniV2 && Robot.config.driveType == DriveType.OperatorMecanum) {
            rightMotor = new Motor(MotorTypes.SRX, "Right Front Drive", 2, -1, logging);
            rightRearMotor = new Motor(MotorTypes.SRX, "Right Rear Drive", 4, -1, logging);
            leftMotor = new Motor(MotorTypes.SRX, "Left Front Drive", 3, -1, logging);
            leftRearMotor = new Motor(MotorTypes.SRX, "Left Rear Drive", 5, -1, logging);

            leftMotor.setInverted(false);
            leftRearMotor.setInverted(false);

            // mecDrive = new MecanumDrive(leftMotor, leftRearMotor, rightMotor,
            // rightRearMotor);
        }

        if (Config.robotType == Config.RobotType.Shooter) {
            rightMotor = new Motor(MotorTypes.SRX, "Right Front Drive", Robot.config.driveRight,
                    Robot.config.driveRightFollow, logging);
            // rightRearMotor = new Motor(MotorTypes.SRX, "Right Rear Drive", 4, 2,
            // logging);
            leftMotor = new Motor(MotorTypes.SRX, "Left Front Drive", Robot.config.driveLeft,
                    Robot.config.driveLeftFollow, logging);
            // leftRearMotor = new Motor(MotorTypes.SRX, "Left Rear Drive", 5, 3, logging);

            leftMotor.setInverted(false);
            // leftRearMotor.setInverted(false);

        }
        // if (Config.robotType == Config.RobotType.Competition) {
        // rightMotor = new Motor(MotorTypes.SRX, "Right Drive", 2, 4, logging);
        // leftMotor = new Motor(MotorTypes.SRX, "Left Drive", 3, 5, logging);
        // }

        if (Robot.config.positionPID == null) {
            logf("************ No PID  ******************\n");
        }
        if (rightMotor.type == MotorTypes.SRX) {
            // todo: check if need separate PIDs for Mecanum
            setPositionPID(Robot.config.positionPID);
            setVelocityPID(Robot.config.velocityPID);
        }
        if (rightMotor.type == MotorTypes.SRX) {
            rightMotor.getSRXMotor().setCurrentLimit(30, 35, 200);
            leftMotor.getSRXMotor().setCurrentLimit(30, 35, 200);
            if (rightRearMotor != null) {
                rightRearMotor.getSRXMotor().setCurrentLimit(30, 35, 200);
                leftRearMotor.getSRXMotor().setCurrentLimit(30, 35, 200);
            }
        }

    }

    public int rightEncoder() {
        return rightMotor.getEncoder();
    }

    public int leftEncoder() {
        return leftMotor.getEncoder();
    }

    public void setSpeed(double right, double left) {
        rightMotor.setSpeed(right);
        leftMotor.setSpeed(left);
    }

    // For Mecanum
    public void setSpeed(double right, double rightRear, double left, double leftRear) {
        if (rightRearMotor != null) {
            rightMotor.setSpeed(right);
            rightRearMotor.setSpeed(rightRear);
            leftMotor.setSpeed(left);
            leftRearMotor.setSpeed(leftRear);
        }
    }

    @Override
    public void periodic() {
        rightMotor.periodic("Right");
        leftMotor.periodic("Left");
    }

    // todo try to get motion magic working

    public void setBrakeMode(boolean mode) {
        rightMotor.setBrakeMode(mode);
        leftMotor.setBrakeMode(mode);
    }

    public void setDefaultBrakeMode() {
        setBrakeMode(Robot.config.defaultBrakeMode);
    }

    // Set the initial position of the robot wheels used in later getDistance Inches
    public void setInitialPosition() {
        rightStart = rightEncoder();
        leftStart = leftEncoder();
    }

    // Get the distance the robot has traveled in inches based off encoder ticks
    public double getDistanceInches() {
        double distanceTicks = Math
                .sqrt(Math.pow(rightStart - rightEncoder(), 2) + Math.pow(leftStart - leftEncoder(), 2));
        return distanceTicks / Robot.config.driveTicksPerInch;
    }

    // Needed for the color wheel program (removes ability to drive forward when
    // deploying color wheel arm
    // to prevent it from bending/snapping off)
    public void stopForwardMotion() {
        Robot.neoPixelControl.setLed(NeoPixelControl.LedAssignment.AllowForward, NeoPixelControl.LedType.RED);
        stopForwardMotion = true;
    }

    // Needed for the color wheel program (allows abiility to drive forward when
    // done with color wheel)
    public void allowForwardMotion() {
        Robot.neoPixelControl.setLed(NeoPixelControl.LedAssignment.AllowForward, NeoPixelControl.LedType.GREEN);
        stopForwardMotion = false;
    }

    public boolean getStopForwardMotion() {
        return stopForwardMotion;
    }

    public void driveRobot() {
        if (Robot.config.driveType != DriveType.OperatorMecanum) {

            double rightSpeed = getRightSpeed();
            double leftSpeed = getLeftSpeed();
            if (Robot.count % 20 == 5
                    && (Math.abs(rightSpeed) > Robot.config.deadZone || Math.abs(leftSpeed) > Robot.config.deadZone)) {
                String message = String.format("Set Drive Speed r:%.3f l:%.3f\n", rightSpeed, leftSpeed);
                if (!message.equals(lastMessage)) { // Do a log if the message has changed
                    logf(message);
                    lastMessage = message;
                }
            }
            if (Robot.count % 500 == 0) {
                rightMotor.logMotorVCS();
                leftMotor.logMotorVCS();
            }

            rightMotor.setSpeed(adjustSpeed(rightSpeed));
            leftMotor.setSpeed(adjustSpeed(leftSpeed));
        }

        else { // Mecanum
            strafe();
            double x = getMecX();
            double y = getMecY();
            double z = getMecZ();

            leftMotor.setInverted(false);
            leftRearMotor.setInverted(false);

            mecDrive.driveCartesian(x, y, z);
            // driveCartesian​(double ySpeed, double xSpeed, double zRotation)

            // System.out.println( "Driving: Axis5- " + limitAxis5 + ", Axis4- " +
            // limitAxis4 + ", Turn0- " + limitTurn0 );
            log("Driving: X: " + x + " Y: " + y + " Z: " + z);
            // todo SmartDashboard.putData("Driving: X: " + x + " Y: " + y + " Z: " + z);
        }

    }

    public void setPositionPID(PID pid) { // assign to 0
        if (Config.robotType == RobotType.Shooter) {
            rightMotor.setPositionPID(pid, true);
            leftMotor.setPositionPID(pid, true);
            rightMotor.getSRXMotor().setPeakOutput(.3);
            leftMotor.getSRXMotor().setPeakOutput(.3);
        } else {
            rightMotor.setPositionPID(pid, false);
            leftMotor.setPositionPID(pid, false);
        }
    };

    public void setVelocityPID(PID pid) { // assign to 1
        rightMotor.setVelocityPID(pid);
        leftMotor.setVelocityPID(pid);
    };

    public void zeroEncoders() {
        rightMotor.zeroEncoder();
        leftMotor.zeroEncoder();
    }

    // Set speed considering joystick noise and rates for exponential throttle
    public double adjustSpeedOrig(double speed) {
        if (speed > Robot.config.deadZone || speed < -Robot.config.deadZone)
            return speed;
        return 0;
    }

    // Set speed considering joystick noise and rates for exponential throttle
    // creates an exponential curve which takes the value from the joysticks as x
    // and converts it to power
    // being sent to motor as y... curve is specific to this deadzone -- graph for
    // visual!
    // purpose: very slow increase in speed after dead zone at first, very fast
    // increase
    // as go to full power
    public static double adjustSpeed(double speed) {

        // if the speed is greater than the deadzone, adjust the speed with exponential
        // throttle
        // If robot type is shooter slow it down
        if (Config.robotType == RobotType.Shooter)
            return speed * .3;
        if (speed > Robot.config.deadZone) {
            // adjust speed so end of deadzone is 0 and max is 1
            speed = (speed - Robot.config.deadZone) / maxInput;
            return Math.pow(((Math.pow(2, speed)) - 1), Robot.config.throttleRate);
        } else if (speed < -Robot.config.deadZone) {
            speed = (speed + Robot.config.deadZone) / maxInput;
            return -(Math.pow(((Math.pow(2, ((-1) * speed))) - 1), Robot.config.throttleRate));
            // if the speed is less than the deadzone, set the speed to zero
        } else if (Math.abs(speed) < Robot.config.deadZone) {
            return 0.0;
        }
        return 0.0;
    }

    public double getRightSpeed() {
        if (Robot.config.driveType == DriveType.Tank) {
            double y = Joysticks.rightJoy.getY();
            if (stopForwardMotion && y < 0)
                y = 0;
            return y;
        }
        if (Robot.config.driveType == DriveType.SingleJoy) {
            // Code from Summer 2019 Asher
            return Joysticks.rightJoy.getY() + Joysticks.rightJoy.getX();
        }
        if (Robot.config.driveType == DriveType.OperatorSingleJoy) {
            // Code from Summer 2019 Asher
            double y = Joysticks.operator.getY();
            return y + Joysticks.operator.getX() * ((y > 0) ? -.3 : .3);
        }
        if (Robot.config.driveType == DriveType.OperatorTank) {
            return Joysticks.operator.getRawAxis(5);
        }
        return 0.0;
    }

    public double getLeftSpeed() {
        if (Robot.config.driveType == DriveType.Tank) {
            double y = Joysticks.leftJoy.getY();
            if (stopForwardMotion && y < 0)
                y = 0;
            return -y;
        }
        if (Robot.config.driveType == DriveType.SingleJoy) {
            // Code from Summer 2019 Asher
            return Joysticks.rightJoy.getY() + Joysticks.rightJoy.getX();
        }
        if (Robot.config.driveType == DriveType.OperatorSingleJoy) {
            // Code from Summer 2019 Asher
            double y = Joysticks.operator.getY();
            return -y + Joysticks.operator.getX() * ((y > 0) ? -.3 : .3);
        }
        if (Robot.config.driveType == DriveType.OperatorTank) {
            return -Joysticks.operator.getRawAxis(1);
        }
        return 0.0;
    }

    // For Mecanum
    private double getMecX() {
        return Joysticks.operator.getRawAxis(5);
    }

    private double getMecY() {
        return Joysticks.operator.getRawAxis(4);
    }

    private double getMecZ() {
        return Joysticks.operator.getRawAxis(0);
    }

    public void strafe() {
        Joystick gamepad = Joysticks.operator;
        while (((gamepad.getRawAxis(4) > 0.2) || (gamepad.getRawAxis(4) < -0.2))
                && ((gamepad.getRawAxis(5) < 0.1) && (gamepad.getRawAxis(5) > -0.1))) {
            leftRearMotor.setInverted(true);
            leftMotor.setInverted(true);
            rightMotor.set(gamepad.getRawAxis(4));
            leftMotor.set(-1 * gamepad.getRawAxis(4));
            rightRearMotor.set(-1 * gamepad.getRawAxis(4));
            leftRearMotor.set(gamepad.getRawAxis(4));
            log("Strafing");
            if ((gamepad.getRawAxis(0) > 0.1) || (gamepad.getRawAxis(0) < -0.1)) {
                log("Stopped strafing");
                return;
            }
        }
    }
    //
}
