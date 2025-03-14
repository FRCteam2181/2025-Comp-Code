package frc.robot;


import static edu.wpi.first.units.Units.Rotations;

import edu.wpi.first.units.measure.Angle;
import frc.robot.Constants.AlgaeRotatorConstants;


public class RobotMath
{

  public static class AlgaeRotatorMath
  {

    /**
     * Convert {@link Angle} into motor {@link Angle}
     *
     * @param measurement Angle, to convert.
     * @return {@link Angle} equivalent to rotations of the motor.
     */
    public static Angle convertAlgaeAngleToSensorUnits(Angle measurement)
    {
      return Rotations.of(measurement.in(Rotations) * AlgaeRotatorConstants.kAlgaeArmReduction);
    }

    /**
     * Convert motor rotations {@link Angle} into usable {@link Angle}
     *
     * @param measurement Motor roations
     * @return Usable angle.
     */
    public static Angle convertSensorUnitsToAlgaeAngle(Angle measurement)
    {
      return Rotations.of(measurement.in(Rotations) / AlgaeRotatorConstants.kAlgaeArmReduction);

    }
  }

 }
