package frc.robot.utilities;

import java.text.DateFormat;
import java.text.SimpleDateFormat;
import java.util.Date;
import java.util.TimeZone;
import java.io.File;

import frc.robot.Config;
// import frc.robot.Robot;
import frc.robot.Robot;

public class Util {
    // Logging Methods
    // Use tail -f /home/lvuser/FRC_UserProgram.log | grep '++++\|----\|????' on
    // roboRio to see important commands

    public static void log(String s) {
        DateFormat dateFormat = new SimpleDateFormat("HH:mm:ss-SSS ");
        dateFormat.setTimeZone(TimeZone.getTimeZone("America/New_York"));
        Date date = new Date();
        System.out.println(dateFormat.format(date) + s);
    }

    public static void logf(String pattern, Object... arguments) {
        try {
            DateFormat dateFormat = new SimpleDateFormat("HH:mm:ss-SSS ");
            dateFormat.setTimeZone(TimeZone.getTimeZone("America/New_York"));
            System.out.printf((dateFormat.format(new Date()) + pattern), arguments);
        } catch (Exception e) {
            System.err.println("\nAn error occurred while logging! Pattern: " + pattern);
            e.printStackTrace();
        }
    }

    public static void loginfo(String pattern, Object... arguments) {
        if(Robot.logging) logf(pattern, arguments);
      }
  

    // Rounding Methods

    public static double round0(double d) {
        return Math.round(d);
    }

    public static double round1(double d) {
        return Math.round(d * 10D) / 10D;
    }

    public static double round2(double d) {
        return Math.round(d * 100D) / 100D;
    }

    public static double round3(double d) {
        return Math.round(d * 1000D) / 1000D;
    }

    public static double round4(double d) {
        return Math.round(d * 10000) / 10000.0;
    }

    // Take an angle and convert it to -180 to 180
    public static double normalizeAngle(double angle) {
        double a = (angle + 180) % 360;
        if (a < 0)
            a += 360;
        return a - 180;
    }

    // Take an angle and convert it to 0 to 360
    public static double unNormalilzeAngle(double angle) {
        double a = angle % 360;
        if (a < 0)
            a += 360;
        return a;
    }

    public static String showFileTime() {
        File file = new File("/home/lvuser/robotCommand");
        SimpleDateFormat sdf = new SimpleDateFormat("MM/dd/yyyy HH:mm:ss");
        return sdf.format(file.lastModified());
    }

    public static void splashScreen(String version) {
        Util.log("**********************************************************************");
        Util.log("");
        Util.logf("Robot Type %s Started compiled:%s version:%s \n", Config.robotType, Util.showFileTime(), version);
        Util.log("");
        Util.log("**********************************************************************");
    }

}
