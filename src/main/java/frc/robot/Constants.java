// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
// import frc.robot.RobotMath.AlgaeRotatorMath;
// import frc.robot.RobotMath.AlgaeRotatorMath;
import swervelib.math.Matter;

import static edu.wpi.first.units.Units.*;

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
  public static final double MAX_SPEED  = 5.331370317;
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
    

    public static final double k_AlgaeClawIntakeSpeed = 0.4;
    public static final double k_AlgaeClawProcessorSpeed = 0.75;
    public static final double k_AlgaeClawBargeSpeed = 0.25;
    public static final double k_AlgaeClawRotateSpeed = .30;


    public static final int k_AlgaeClawVoltageLimit = 80;

  }

  public static class AlgaeRotatorConstants {

    public static final int k_AlgaeClawRotatorID = 13;
    
    public static final int     kAlgaeArmStallCurrentLimitAmps  = 40;

    public static final double k_AlgaeClawRotateSpeed = .250;
  }
  
  public static class CoralFunnelConstants {
    public static final int k_CoralFunnelWheelID = 15;
    public static final int k_CoralRotatorID = 14;
    
    public static final double k_CoralFunnelSpeed = -.20;
    public static final double k_CoralFunnelSpeedext = -.80;
    public static final double k_FunnelRotateSpeed = 0.1;
    
    public static final int k_CoralFunnelVoltageLimit = 80;

    public static final int coralSensorId = 22;
    public static final double coralDistanceThreshold = 100;//TODO Change



  
  }

  public static class climberConstants{
    public static final int k_climberID = 18;
    public static final double k_climberSpeedUp = 1;
    
    public static final double k_ClimberSpeedDown = 1;

    public static final int k_climberVoltageLimit = 80;  


  }

  public static class Colors {
    //Basic Blinkin patterns (section 5): https://www.revrobotics.com/content/docs/REV-11-1105-UM.pdf
    
    public static class fixedPalettePattern {
      public double rainbow_rainbow = -0.99;
      public double rainbow_party = -0.97;
      public double rainbow_ocean = -0.95;
      public double rainbow_lava = -0.93;
      public double rainbow_forest = -0.91;
      public double rainbow_glitter = -0.89;
      public double confetti = -0.87;
      public double shot_red = -0.85;
      public double shot_blue = -0.83;
      public double shot_white = -0.81;
      public double sinelon_rainbow = -0.79;
      public double sinelon_party = -0.77;
      public double sinelon_ocean = -0.75;
      public double sinelon_lava = -0.73;
      public double sinelon_forest = -0.71;
      public double bpm_rainbow = -0.69;
      public double bpm_party = -0.67;  
      public double bpm_ocean = -0.65;
      public double bmp_lava = -0.63;
      public double bpm_forest = -0.61;
      public double fire_medium = -0.59;
      public double fire_large = -0.57;  
      public double twinkle_rainbow = -0.55;
      public double twinkle_party = -0.53;
      public double twinkle_ocean = -0.51;
      public double twinkle_lava = -0.49;
      public double twinkle_forest = -0.47;
      public double waves_rainbow = -0.45;
      public double waves_party = -0.43;
      public double waves_ocean = -0.41;
      public double waves_lava = -0.39;
      public double waves_forest = -0.37;
      public double larson_red = -0.35;
      public double larson_gray = -0.33;
      public double chase_red = -0.31;
      public double chase_blue = -0.29;
      public double chase_gray = -0.27;
      public double heartbeat_red = -0.25;
      public double heartbeat_blue = -0.23;
      public double heartbeat_white = -0.21;
      public double heartbeat_gray = -0.19;
      public double breath_red = -0.17;  
      public double breath_blue = -0.15;
      public double breath_gray = -0.13;
      public double strobe_red = -0.11;
      public double strobe_blue = -0.09;
      public double strobe_gold = -0.07;  
      public double strobe_white = -0.05;
    }

    public static class color1Pattern {
      public double black_blend = -0.03;
      public double larson = -0.01;
      public double chase = 0.01;
      public double heartbeat_slow = 0.03;
      public double heartbeat_medium = 0.05;
      public double heartbeat_fast = 0.07;
      public double breath_slow = 0.09;
      public double breath_fast = 0.11;
      public double shot = 0.13;
      public double strobe = 0.15;
    }

    public static class color2Pattern {
      public double blend_black = 0.17;
      public double larson = 0.19;
      public double chase = 0.21;
      public double heartbeat_slow = 0.23;
      public double heartbeat_medium = 0.25;
      public double heartbeat_fast = 0.27;
      public double breath_slow = 0.29;
      public double breath_fast = 0.31;
      public double shot = 0.33;
      public double strobe = 0.35;
    }

    public static class color1and2Pattern {
      public double sparkel_1on2 = 0.37;
      public double sparkle_2on1 = 0.39;
      public double gradient_1and2 = 0.41;
      public double bmp_1and2 = 0.43;
      public double bleed_1to2 = 0.45;
      public double bleed_2to1 = 0.47;
      public double noblend_1and2 = 0.49;
      public double twinkles_1and2 = 0.51;
      public double waves_1and2 = 0.53;
      public double sinelon_1and2 = 0.55;
    }

    public static class solidColors {
      public double hot_pink = 0.57;
      public double dark_red = 0.59;
      public double red = 0.61;
      public double red_orange = 0.63;
      public double orange = 0.65;
      public double gold = 0.67;
      public double yellow = 0.69;
      public double lawn_green = 0.71;
      public double lime = 0.73;
      public double dark_green = 0.75;
      public double green = 0.77;
      public double blue_green = 0.79;
      public double aqua = 0.81;
      public double sky_blue = 0.83;
      public double dark_blue = 0.85;
      public double blue = 0.87;
      public double blue_violet = 0.89;
      public double violet = 0.91;
      public double white = 0.93;
      public double gray = 0.95;
      public double dark_gray = 0.97;
      public double black = 0.9;
    }
        
  }





  // public static class targetingConstants
  // {

  //   public static final double positiveScootch = Units.inchesToMeters(5);
  //   public static final double negitiveScootch = Units.inchesToMeters(-5);
  //   public static final double scootchBack     = Units.inchesToMeters(12);
  // }

  



      public static class ElevatorConstants
      {
    
        public static final double   kElevatorKp              = 22;
        public static final double   kElevatorKi              = 0;
        public static final double   kElevatorKd              = 1.5;
        
        public static final double   kElevatorkS              = 0;//0.01964; // volts (V)
        public static final double   kElevatorkV              = 0;//2.63; // volt per velocity (V/(m/s))
        public static final double   kElevatorkA              = 0;//0.14; // volt per acceleration (V/(m/s²))
        public static final double   kElevatorkG              = 0;//0.91274; // volts (V)
       
        public static final double   kElevatorGearing         = 12.0; 
        public static final double   kElevatorDrumDiameter      = Units.inchesToMeters(1.751);
       
        
        
        // Encoder is reset to measure 0 at the bottom, so minimum height is 0.
        public static final double   kMinElevatorHeightMeters = 0;//min height 
        public static final double   kMaxElevatorHeightMeters = 10.25;
        public static final Distance kMinElevatorHeight      = Meters.of(kMinElevatorHeightMeters);
        public static final Distance kMaxElevatorHeight      = Meters.of(kMaxElevatorHeightMeters);
        public static final double   kElevatorAllowableError = .04;  //.04
        public static final double   kLowerToScoreHeight     = Units.inchesToMeters(6);
        
        public static       double   kElevatorRampRate       = 0.1;
        public static       int      kElevatorCurrentLimit   = 40;
        public static double kMaxVelocity = Meters.of(13).per(Second).in(MetersPerSecond);
        public static double kMaxAcceleration = Meters.of(13).per(Second).per(Second).in(MetersPerSecondPerSecond);
        public static final double   kElevatorUnextendedHeight    = Units.inchesToMeters(41.5);



        public static final double k_FeederStation = Units.inchesToMeters(17.375);
        public static final double k_L1 = Units.inchesToMeters(0);
        public static final double k_L2 = Units.inchesToMeters(27.75);
        public static final double k_L3 = Units.inchesToMeters(43.625);
        public static final double k_L4 = Units.inchesToMeters(68.875);

        public static final double k_Processor = 0;
        public static final double k_AGround = 0;
        public static final double k_A1 = Units.inchesToMeters(15.5);
        public static final double k_A2 = Units.inchesToMeters(31);
        public static final double k_Net = Units.inchesToMeters(73.875);
        
        public static final int k_ElevatorLeftID = 17;
        public static final int k_ElevatorRightID = 16;

        public static final Distance kLaserCANOffset          = Meters.of(0);//TODO Change
        public static final int elevatorLaserCanID  = 19;
      }


}
