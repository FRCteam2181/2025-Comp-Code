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

    public static final double k_AlgaeClawIntakeSpeed = 0.4;
    public static final double k_AlgaeClawProcessorSpeed = 0.5;
    public static final double k_AlgaeClawBargeSpeed = 0.25;
    public static final double k_AlgaeClawRotateSpeed = .30;


    public static final int k_AlgaeClawVoltageLimit = 80;

  }
  
  public static class CoralFunnelConstants {
    public static final int k_CoralFunnelWheelID = 15;
    public static final int k_CoralRotatorID = 14;
    
    public static final double k_CoralFunnelSpeed = -.15;
    public static final double k_CoralFunnelSpeedext = -.80;
    public static final double k_FunnelRotateSpeed = 0.2;
    
    public static final int k_CoralFunnelVoltageLimit = 80;
  
  }
  public static class climberConstants{
    public static final int m_climberID = 18;
    public static final double m_climberSpeedUp = 1;
    
    public static final double k_ClimberSpeedDown = 1;

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





  public static class targetingConstants
  {

    public static final double positiveScootch = Units.inchesToMeters(5);
    public static final double negitiveScootch = Units.inchesToMeters(-5);
    public static final double scootchBack     = Units.inchesToMeters(12);
  }

  



      public static class ElevatorConstants
      {
    
        public static final double   kElevatorKp              = 5;
        public static final double   kElevatorKi              = 0;
        public static final double   kElevatorKd              = 0;
        
        public static final double   kElevatorkS              = 0; // volts (V)
        public static final double   kElevatorkV              = 0; // volt per velocity (V/(m/s))
        public static final double   kElevatorkA              = 0; // volt per acceleration (V/(m/s²))
        public static final double   kElevatorkG              = 0; // volts (V)
       
        public static final double   kElevatorGearing         = 5.0; // ours should be 5
        public static final double   kElevatorDrumDiameter      = Units.inchesToMeters(1.751);
       
        
        
        // Encoder is reset to measure 0 at the bottom, so minimum height is 0.
        public static final double   kMinElevatorHeightMeters = 0;//min height / 10
        public static final double   kMaxElevatorHeightMeters = 10.25;
        public static final Distance kMinElevatorHeight      = Meters.of(kMinElevatorHeightMeters);
        public static final Distance kMaxElevatorHeight      = Meters.of(kMaxElevatorHeightMeters);
        public static final double   kElevatorAllowableError = 1;
        public static final double   kLowerToScoreHeight     = Units.inchesToMeters(6);
        
        public static       double   kElevatorRampRate       = 0.1;
        public static       int      kElevatorCurrentLimit   = 40;
        public static double kMaxVelocity = Meters.of(4).per(Second).in(MetersPerSecond);
        public static double kMaxAcceleration = Meters.of(8).per(Second).per(Second).in(MetersPerSecondPerSecond);
        public static final double   kElevatorUnextendedHeight    = Units.inchesToMeters(41.5);



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
        
        public static final int k_ElevatorLeftID = 17;
        public static final int k_ElevatorRightID = 16;


      }



    //Pose to align with Each Reef branch

      public static class TargetingConstants {
        public static final Pose2d ReefBranch_Red_A = new Pose2d(
          Units.inchesToMeters(0),
          Units.inchesToMeters(0),
          Rotation2d.fromDegrees(0));

        public static final Pose2d ReefBranch_Red_AB_Middle = new Pose2d(
            Units.inchesToMeters(0),
            Units.inchesToMeters(0),
            Rotation2d.fromDegrees(0));

        public static final Pose2d ReefBranch_Red_B = new Pose2d(
          Units.inchesToMeters(0),
          Units.inchesToMeters(0),
          Rotation2d.fromDegrees(0));

        public static final Pose2d ReefBranch_Red_C = new Pose2d(
          Units.inchesToMeters(0),
          Units.inchesToMeters(0),
          Rotation2d.fromDegrees(0));

        public static final Pose2d ReefBranch_Red_CD_Middle = new Pose2d(
          Units.inchesToMeters(0),
          Units.inchesToMeters(0),
          Rotation2d.fromDegrees(0));

        public static final Pose2d ReefBranch_Red_D = new Pose2d(
          Units.inchesToMeters(0),
          Units.inchesToMeters(0),
          Rotation2d.fromDegrees(0));

        public static final Pose2d ReefBranch_Red_E = new Pose2d(
          Units.inchesToMeters(0),
          Units.inchesToMeters(0),
          Rotation2d.fromDegrees(0));

        public static final Pose2d ReefBranch_Red_EF_Middle = new Pose2d(
          Units.inchesToMeters(0),
          Units.inchesToMeters(0),
          Rotation2d.fromDegrees(0));

        public static final Pose2d ReefBranch_Red_F = new Pose2d(
          Units.inchesToMeters(0),
          Units.inchesToMeters(0),
          Rotation2d.fromDegrees(0));

        public static final Pose2d ReefBranch_Red_G = new Pose2d(
          Units.inchesToMeters(0),
          Units.inchesToMeters(0),
          Rotation2d.fromDegrees(0));

        public static final Pose2d ReefBranch_Red_GH_Middle = new Pose2d(
          Units.inchesToMeters(0),
          Units.inchesToMeters(0),
          Rotation2d.fromDegrees(0));

        public static final Pose2d ReefBranch_Red_H = new Pose2d(
          Units.inchesToMeters(0),
          Units.inchesToMeters(0),
          Rotation2d.fromDegrees(0));

        public static final Pose2d ReefBranch_Red_I = new Pose2d(
          Units.inchesToMeters(0),
          Units.inchesToMeters(0),
          Rotation2d.fromDegrees(0));

        public static final Pose2d ReefBranch_Red_IJ_Middle = new Pose2d(
          Units.inchesToMeters(0),
          Units.inchesToMeters(0),
          Rotation2d.fromDegrees(0));

        public static final Pose2d ReefBranch_Red_J = new Pose2d(
          Units.inchesToMeters(0),
          Units.inchesToMeters(0),
          Rotation2d.fromDegrees(0));

        public static final Pose2d ReefBranch_Red_K = new Pose2d(
          Units.inchesToMeters(0),
          Units.inchesToMeters(0),
          Rotation2d.fromDegrees(0));

          public static final Pose2d ReefBranch_Red_KL_Middle = new Pose2d(
            Units.inchesToMeters(0),
            Units.inchesToMeters(0),
            Rotation2d.fromDegrees(0));

        public static final Pose2d ReefBranch_Red_L = new Pose2d(
          Units.inchesToMeters(0),
          Units.inchesToMeters(0),
          Rotation2d.fromDegrees(0));





        public static final Pose2d ReefBranch_Blue_A = new Pose2d(
          Units.inchesToMeters(0),
          Units.inchesToMeters(0),
          Rotation2d.fromDegrees(0));

        public static final Pose2d ReefBranch_Blue_AB_Middle = new Pose2d(
          Units.inchesToMeters(0),
          Units.inchesToMeters(0),
          Rotation2d.fromDegrees(0));

        public static final Pose2d ReefBranch_Blue_B = new Pose2d(
          Units.inchesToMeters(0),
          Units.inchesToMeters(0),
          Rotation2d.fromDegrees(0));

        public static final Pose2d ReefBranch_Blue_C = new Pose2d(
          Units.inchesToMeters(0),
          Units.inchesToMeters(0),
          Rotation2d.fromDegrees(0));

        public static final Pose2d ReefBranch_Blue_CD_Middle = new Pose2d(
          Units.inchesToMeters(0),
          Units.inchesToMeters(0),
          Rotation2d.fromDegrees(0));

        public static final Pose2d ReefBranch_Blue_D = new Pose2d(
          Units.inchesToMeters(0),
          Units.inchesToMeters(0),
          Rotation2d.fromDegrees(0));

        public static final Pose2d ReefBranch_Blue_E = new Pose2d(
          Units.inchesToMeters(0),
          Units.inchesToMeters(0),
          Rotation2d.fromDegrees(0));
        
        public static final Pose2d ReefBranch_Blue_EF_Middle = new Pose2d(
          Units.inchesToMeters(0),
          Units.inchesToMeters(0),
          Rotation2d.fromDegrees(0));

        public static final Pose2d ReefBranch_Blue_F = new Pose2d(
          Units.inchesToMeters(0),
          Units.inchesToMeters(0),
          Rotation2d.fromDegrees(0));

        public static final Pose2d ReefBranch_Blue_G = new Pose2d(
          Units.inchesToMeters(0),
          Units.inchesToMeters(0),
          Rotation2d.fromDegrees(0));
        
        public static final Pose2d ReefBranch_Blue_GH_Middle = new Pose2d(
          Units.inchesToMeters(0),
          Units.inchesToMeters(0),
          Rotation2d.fromDegrees(0));

        public static final Pose2d ReefBranch_Blue_H = new Pose2d(
          Units.inchesToMeters(0),
          Units.inchesToMeters(0),
          Rotation2d.fromDegrees(0));

        public static final Pose2d ReefBranch_Blue_I = new Pose2d(
          Units.inchesToMeters(0),
          Units.inchesToMeters(0),
          Rotation2d.fromDegrees(0));

          public static final Pose2d ReefBranch_Blue_IJ_Middle = new Pose2d(
            Units.inchesToMeters(0),
            Units.inchesToMeters(0),
            Rotation2d.fromDegrees(0));

        public static final Pose2d ReefBranch_Blue_J = new Pose2d(
          Units.inchesToMeters(0),
          Units.inchesToMeters(0),
          Rotation2d.fromDegrees(0));

        public static final Pose2d ReefBranch_Blue_K = new Pose2d(
          Units.inchesToMeters(0),
          Units.inchesToMeters(0),
          Rotation2d.fromDegrees(0));

        public static final Pose2d ReefBranch_Blue_KL_Middle = new Pose2d(
          Units.inchesToMeters(0),
          Units.inchesToMeters(0),
          Rotation2d.fromDegrees(0));

        public static final Pose2d ReefBranch_Blue_L = new Pose2d(
          Units.inchesToMeters(0),
          Units.inchesToMeters(0),
          Rotation2d.fromDegrees(0));




    //Reef Heights for Elevator in meters
   
        public static final Distance ReefBranchHeight_L1 = Meters.of(0);
        public static final Distance ReefBranchHeight_L2 = Meters.of(0);
        public static final Distance ReefBranchHeight_L3 = Meters.of(0);
        public static final Distance ReefBranchHeight_L4 = Meters.of(0);
        public static final Distance ReefBranchHeight_A_Low = Meters.of(0);
        public static final Distance ReefBranchHeight_A_High = Meters.of(0);
   


    //Pose to align to in front of cage
        public static final Pose2d DesiredCage_Red_Cage_1 = new Pose2d(
          Units.inchesToMeters(0),
          Units.inchesToMeters(0),
          Rotation2d.fromDegrees(0));
          
        public static final Pose2d DesiredCage_Red_Cage_2 = new Pose2d(
          Units.inchesToMeters(0),
          Units.inchesToMeters(0),
          Rotation2d.fromDegrees(0));      

        public static final Pose2d DesiredCage_Red_Cage_3 = new Pose2d(
          Units.inchesToMeters(0),
          Units.inchesToMeters(0),
          Rotation2d.fromDegrees(0));
          
        public static final Pose2d DesiredCage_Blue_Cage_1 = new Pose2d(
          Units.inchesToMeters(0),
          Units.inchesToMeters(0),
          Rotation2d.fromDegrees(0));              
      
        public static final Pose2d DesiredCage_Blue_Cage_2 = new Pose2d(
          Units.inchesToMeters(0),
          Units.inchesToMeters(0),
          Rotation2d.fromDegrees(0));
          
        public static final Pose2d DesiredCage_Blue_Cage_3 = new Pose2d(
          Units.inchesToMeters(0),
          Units.inchesToMeters(0),
          Rotation2d.fromDegrees(0));    


    //Pose to align to in front of processor

        public static final Pose2d Red_Processor = new Pose2d(
          Units.inchesToMeters(0),
          Units.inchesToMeters(0),
          Rotation2d.fromDegrees(0));
          
        public static final Pose2d Blue_Processor = new Pose2d(
          Units.inchesToMeters(0),
          Units.inchesToMeters(0),
          Rotation2d.fromDegrees(0));  


    //Pose to align to in front of each coral feeder station slot


        public static final Pose2d CoralFeed_Red_Left_1 = new Pose2d(
          Units.inchesToMeters(0),
          Units.inchesToMeters(0),
          Rotation2d.fromDegrees(0));

        public static final Pose2d CoralFeed_Red_Left_2 = new Pose2d(
          Units.inchesToMeters(0),
          Units.inchesToMeters(0),
          Rotation2d.fromDegrees(0));

        public static final Pose2d CoralFeed_Red_Left_3 = new Pose2d(
          Units.inchesToMeters(0),
          Units.inchesToMeters(0),
          Rotation2d.fromDegrees(0));

        public static final Pose2d CoralFeed_Red_Left_4 = new Pose2d(
          Units.inchesToMeters(0),
          Units.inchesToMeters(0),
          Rotation2d.fromDegrees(0));

        public static final Pose2d CoralFeed_Red_Left_5 = new Pose2d(
          Units.inchesToMeters(0),
          Units.inchesToMeters(0),
          Rotation2d.fromDegrees(0));

        public static final Pose2d CoralFeed_Red_Left_6 = new Pose2d(
          Units.inchesToMeters(0),
          Units.inchesToMeters(0),
          Rotation2d.fromDegrees(0));

        public static final Pose2d CoralFeed_Red_Left_7 = new Pose2d(
          Units.inchesToMeters(0),
          Units.inchesToMeters(0),
          Rotation2d.fromDegrees(0));

        public static final Pose2d CoralFeed_Red_Left_8 = new Pose2d(
          Units.inchesToMeters(0),
          Units.inchesToMeters(0),
          Rotation2d.fromDegrees(0));

        public static final Pose2d CoralFeed_Red_Left_9 = new Pose2d(
          Units.inchesToMeters(0),
          Units.inchesToMeters(0),
          Rotation2d.fromDegrees(0));

        public static final Pose2d CoralFeed_Red_Right_1 = new Pose2d(
          Units.inchesToMeters(0),
          Units.inchesToMeters(0),
          Rotation2d.fromDegrees(0));

        public static final Pose2d CoralFeed_Red_Right_2 = new Pose2d(
          Units.inchesToMeters(0),
          Units.inchesToMeters(0),
          Rotation2d.fromDegrees(0));

        public static final Pose2d CoralFeed_Red_Right_3 = new Pose2d(
          Units.inchesToMeters(0),
          Units.inchesToMeters(0),
          Rotation2d.fromDegrees(0));

        public static final Pose2d CoralFeed_Red_Right_4 = new Pose2d(
          Units.inchesToMeters(0),
          Units.inchesToMeters(0),
          Rotation2d.fromDegrees(0));

        public static final Pose2d CoralFeed_Red_Right_5 = new Pose2d(
          Units.inchesToMeters(0),
          Units.inchesToMeters(0),
          Rotation2d.fromDegrees(0));

        public static final Pose2d CoralFeed_Red_Right_6 = new Pose2d(
          Units.inchesToMeters(0),
          Units.inchesToMeters(0),
          Rotation2d.fromDegrees(0));

        public static final Pose2d CoralFeed_Red_Right_7 = new Pose2d(
          Units.inchesToMeters(0),
          Units.inchesToMeters(0),
          Rotation2d.fromDegrees(0));

        public static final Pose2d CoralFeed_Red_Right_8 = new Pose2d(
          Units.inchesToMeters(0),
          Units.inchesToMeters(0),
          Rotation2d.fromDegrees(0));

        public static final Pose2d CoralFeed_Red_Right_9 = new Pose2d(
          Units.inchesToMeters(0),
          Units.inchesToMeters(0),
          Rotation2d.fromDegrees(0));


      

          public static final Pose2d CoralFeed_Blue_Left_1 = new Pose2d(
            Units.inchesToMeters(0),
            Units.inchesToMeters(0),
            Rotation2d.fromDegrees(0));
  
          public static final Pose2d CoralFeed_Blue_Left_2 = new Pose2d(
            Units.inchesToMeters(0),
            Units.inchesToMeters(0),
            Rotation2d.fromDegrees(0));
  
          public static final Pose2d CoralFeed_Blue_Left_3 = new Pose2d(
            Units.inchesToMeters(0),
            Units.inchesToMeters(0),
            Rotation2d.fromDegrees(0));
  
          public static final Pose2d CoralFeed_Blue_Left_4 = new Pose2d(
            Units.inchesToMeters(0),
            Units.inchesToMeters(0),
            Rotation2d.fromDegrees(0));
  
          public static final Pose2d CoralFeed_Blue_Left_5 = new Pose2d(
            Units.inchesToMeters(0),
            Units.inchesToMeters(0),
            Rotation2d.fromDegrees(0));
  
          public static final Pose2d CoralFeed_Blue_Left_6 = new Pose2d(
            Units.inchesToMeters(0),
            Units.inchesToMeters(0),
            Rotation2d.fromDegrees(0));
  
          public static final Pose2d CoralFeed_Blue_Left_7 = new Pose2d(
            Units.inchesToMeters(0),
            Units.inchesToMeters(0),
            Rotation2d.fromDegrees(0));
  
          public static final Pose2d CoralFeed_Blue_Left_8 = new Pose2d(
            Units.inchesToMeters(0),
            Units.inchesToMeters(0),
            Rotation2d.fromDegrees(0));
  
          public static final Pose2d CoralFeed_Blue_Left_9 = new Pose2d(
            Units.inchesToMeters(0),
            Units.inchesToMeters(0),
            Rotation2d.fromDegrees(0));
  
          public static final Pose2d CoralFeed_Blue_Right_1 = new Pose2d(
            Units.inchesToMeters(0),
            Units.inchesToMeters(0),
            Rotation2d.fromDegrees(0));
  
          public static final Pose2d CoralFeed_Blue_Right_2 = new Pose2d(
            Units.inchesToMeters(0),
            Units.inchesToMeters(0),
            Rotation2d.fromDegrees(0));
  
          public static final Pose2d CoralFeed_Blue_Right_3 = new Pose2d(
            Units.inchesToMeters(0),
            Units.inchesToMeters(0),
            Rotation2d.fromDegrees(0));
  
          public static final Pose2d CoralFeed_Blue_Right_4 = new Pose2d(
            Units.inchesToMeters(0),
            Units.inchesToMeters(0),
            Rotation2d.fromDegrees(0));
  
          public static final Pose2d CoralFeed_Blue_Right_5 = new Pose2d(
            Units.inchesToMeters(0),
            Units.inchesToMeters(0),
            Rotation2d.fromDegrees(0));
  
          public static final Pose2d CoralFeed_Blue_Right_6 = new Pose2d(
            Units.inchesToMeters(0),
            Units.inchesToMeters(0),
            Rotation2d.fromDegrees(0));
  
          public static final Pose2d CoralFeed_Blue_Right_7 = new Pose2d(
            Units.inchesToMeters(0),
            Units.inchesToMeters(0),
            Rotation2d.fromDegrees(0));
  
          public static final Pose2d CoralFeed_Blue_Right_8 = new Pose2d(
            Units.inchesToMeters(0),
            Units.inchesToMeters(0),
            Rotation2d.fromDegrees(0));
  
          public static final Pose2d CoralFeed_Blue_Right_9 = new Pose2d(
            Units.inchesToMeters(0),
            Units.inchesToMeters(0),
            Rotation2d.fromDegrees(0));


      }







}
