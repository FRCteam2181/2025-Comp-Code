package frc.robot;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Rotations;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import frc.robot.Constants.AlgaeRotatorConstants;
import frc.robot.Constants.ElevatorConstants;


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


  public static class Elevator
  {

    /**
     * Convert {@link Distance} into {@link Angle}
     *
     * @param distance Distance, usually Meters.
     * @return {@link Angle} equivalent to rotations of the motor.
     */
    public static Angle convertDistanceToRotations(Distance distance)
    {
      // m/(2*pi*r)*g = e
      return Rotations.of((distance.in(Meters) /
                          (Math.PI * ElevatorConstants.kElevatorDrumDiameter)*3) *
                          ElevatorConstants.kElevatorGearing);
    }

    /**
     * Convert {@link Angle} into {@link Distance}
     *
     * @param rotations Rotations of the motor
     * @return {@link Distance} of the elevator.
     */
    public static Distance convertRotationsToDistance(Angle rotations)
    {
      return Meters.of((rotations.in(Rotations) / ElevatorConstants.kElevatorGearing) *
                       ((Math.PI * ElevatorConstants.kElevatorDrumDiameter)*3));
    }
  }






 }
