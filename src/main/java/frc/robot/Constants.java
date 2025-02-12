// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import swervelib.math.Matter;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean constants. This
 * class should not be used for any other purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants
{

  public static final double ROBOT_MASS = (148 - 20.3) * 0.453592; // 32lbs * kg per pound
  public static final Matter CHASSIS    = new Matter(new Translation3d(0, 0, Units.inchesToMeters(8)), ROBOT_MASS);
  public static final double LOOP_TIME  = 0.13; //s, 20ms + 110ms sprk max velocity lag
  public static final double MAX_SPEED  = Units.feetToMeters(14.5);
  // Maximum speed of the robot in meters per second, used to limit acceleration.

//  public static final class AutonConstants
//  {
//
//    public static final PIDConstants TRANSLATION_PID = new PIDConstants(0.7, 0, 0);
//    public static final PIDConstants ANGLE_PID       = new PIDConstants(0.4, 0, 0.01);
//  }

  public static final class DrivebaseConstants
  {

    // Hold time on motor brakes when disabled
    public static final double WHEEL_LOCK_TIME = 10; // seconds
  }

  public static class OperatorConstants
  {

    // Joystick Deadband
    public static final double DEADBAND        = 0.1;
    public static final double LEFT_Y_DEADBAND = 0.1;
    public static final double RIGHT_X_DEADBAND = 0.1;
    public static final double TURN_CONSTANT    = 6;
  }

  public static class ElevatorConstants {

    public static final int k_ElevatorLeftID = 17;
    public static final int k_ElevatorRightID = 16;

    public static final double k_ElevatorSpeed = -0.4;

    public static final int k_FeederStation = 0;
    public static final int k_L1 = 0;
    public static final int k_L2 = 5;
    public static final int k_L3 = 100;
    public static final int k_L4 = 150;

    public static final int k_Processor = 0;
    public static final int k_AGround = 0;
    public static final int k_A1 = 0;
    public static final int k_A2 = 0;
    public static final int k_Net = 0;
    
  }


  public static class CoralPlacerConstants {
    public static final int k_CoralWheelLeftID = 11;
    public static final int k_CoralWheelRightID = 12;

    public static final double k_CoralPlacerSpeedL1 = .4;
    public static final double k_CoralPlacerSpeedGen = 0.6;
    
    public static final int k_CoralPlacerVoltageLimit = 20;
  }

  
  public static class AlgaeClawConstants {

    public static final int k_AlgaeClawTopID = 10;
    public static final int k_AlgaeClawBottomID = 9;
    public static final int k_AlgaeClawRotator = 13;

    public static final double k_AlgaeClawIntakeSpeed = 0.1;
    public static final double k_AlgaeClawProcessorSpeed = 0.5;
    public static final double k_AlgaeClawBargeSpeed = 0.25;
    public static final double k_AlgaeClawRotateSpeed = .10;


    public static final int k_AlgaeClawVoltageLimit = 80;

  }
  
  public static class CoralFunnelConstants {
    public static final int k_CoralFunnelWheelID = 15;
    public static final int k_CoralRotatorID = 14;
    
    public static final double k_CoralFunnelSpeed = -.15;
    public static final double k_CoralFunnelSpeedext = -.80;
    public static final double k_FunnelRotateSpeed = 0.05;
    
    public static final int k_CoralFunnelVoltageLimit = 80;
  
  }
  public static class climberConstants{
    public static final int m_climberID = 19;
    public static final double m_climberSpeed = 50;
    public static final int m_climberVoltageLimit = 80;  


  }

  public static class Colors {
    public double pat1_larscan = -0.01;
    public double pat2_larScan = 0.19;
    public double fix_rain = -0.99;
    public double fix_rainParty = -0.97;
    public double fix_ocean = -0.95;
    public double fix_Lave = -0.93;
    public double fix_forest = -0.91;
    public double endToEndBlend = 0.47;
    public double pat2_lightChase = 0.21;
    public double pat2_shot = 0.33;
    
    public double hotPink = 0.57;
    public double darkRed = 0.59;
    public double red = 0.61;
    public double redOrange = 0.63;
    public double orange = 0.65;
    public double gold = 0.67;
    public double yellow = 0.69;
    public double lawnGreen = 0.71;
    public double lime = 0.73;
    public double darkGreen = 0.75;
    public double green = 0.77;
    public double blue_green = 0.79;
    public double aqua = 0.81;
    public double skyBlue = 0.83;
    public double dark_blue = 0.85;
    public double blue = 0.87;
    public double blueViolet = 0.89;
    public double purple = 0.91;
    public double white = 0.93;
    public double gray = 0.95;
    public double dark_gray = 0.97;
    public double black = 0.99;
    public double OceanWaves = -0.41;
    public double RedChase = -0.31;    
  }









}
