/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.kauailabs.navx.frc.AHRS;

/**
 * An example subsystem. You can replace me with your own Subsystem.
 */
public class YawProvider extends SubsystemBase {
  private final AHRS ahrs;

  public YawProvider() {
    ahrs = new AHRS(SPI.Port.kMXP);
  }

  /**
   * Returns the total accumulated yaw angle in <i>degrees</i> reported by the
   * sensor based on integration of the returned rate from the Z-axis gyro.
   * <p>
   * NOTE: The angle is continuous (meaning its range is beyond 360 degrees),
   * ensuring that there are no discontinuities.
   * <p>
   * This value can be zeroed by calling zeroYaw();
   *
   * @return The current total accumulated yaw angle (Z axis) of the robot in
   *         degrees.
   */

  public double getContinuousYaw() {
    return ahrs.getAngle();
  }

  public double yaw() {
    double yaw = ahrs.getYaw();
    return yaw;
  }


  /**
   * @return The current rate of change of the yaw in <i>degrees per second</i>.
   */
  public double getYawRate() {
    return ahrs.getRate();
  }

  /**
   * Sets the current yaw to be the center, or 0, yaw. This is usually (the NavX
   * board could have yaw reset capabilities) done by setting an offset value to
   * the current yaw that will be subtracted from subsequent yaws.
   */

  public void zeroYaw() {
    //Util.logf("Zero yaw -- previous yaw:%.3f\n", Robot.yaw);
    ahrs.zeroYaw();
  }

  public double getAccelX() {
    return ahrs.getWorldLinearAccelX();
  }

  public double getAccelY() {
    return ahrs.getWorldLinearAccelY();
  }

  public double getAccelZ() {
    return ahrs.getWorldLinearAccelZ();
  }

  /**
   * @return Temperature in degrees Celsius.
   */
  public float getTemperature() {
    return ahrs.getTempC();
  }
}