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
import frc.robot.RobotMath.AlgaeRotatorMath;
import frc.robot.RobotMath.AlgaeRotatorMath;
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
    

    public static final double k_AlgaeClawIntakeSpeed = 0.4;
    public static final double k_AlgaeClawProcessorSpeed = 0.75;
    public static final double k_AlgaeClawBargeSpeed = 0.25;
    public static final double k_AlgaeClawRotateSpeed = .30;


    public static final int k_AlgaeClawVoltageLimit = 80;

  }

  public static class AlgaeRotatorConstants {

    public static final int k_AlgaeClawRotatorID = 13;
    // The P gain for the PID controller that drives this arm.
    public static final double  kAlgaeArmKp                     = 0.005;
    public static final double  kAlgaeArmKi                     = 0;
    public static final double  kAlgaeArmKd                     = .00;
    public static final double  kAlgaeArmReduction              = 180;
    public static final Angle   kAlgaeArmAllowedClosedLoopError
                                                                = AlgaeRotatorMath.convertAlgaeAngleToSensorUnits(Degrees.of(
        1));
    public static final double  kAlgaeArmMass                   = Units.lbsToKilograms(15); // Kilograms
    public static final double  kAlgaeArmLength                 = Inches.of(31).in(Meters);//.7meter
    public static final Angle   kAlgaeArmStartingAngle          = Degrees.of(0);
    public static final Angle   kAlgaeArmMinAngle               = Degrees.of(-45);
    public static final Angle   kAlgaeArmMaxAngle               = Degrees.of(250);
    public static final double  kAlgaeArmRampRate               = 0.5;
    public static final Angle   kAlgaeArmOffsetToHorizantalZero = Rotations.of(0);
    public static final boolean kAlgaeArmInverted               = false;
    public static final double  kAlgaeArmMaxVelocityRPM
                                                                = AlgaeRotatorMath.convertAlgaeAngleToSensorUnits(Degrees.of(
                                                                                            90)).per(
                                                                                            Second).in(RPM);
    public static final double  kAlgaeArmMaxAccelerationRPMperSecond
                                                                = AlgaeRotatorMath.convertAlgaeAngleToSensorUnits(Degrees.of(
                                                                              180)).per(
                                                                              Second).per(Second)
                                                                          .in(RPM.per(Second));
    public static final int     kAlgaeArmStallCurrentLimitAmps  = 40;
    public static final double  kAlgaeArmkS                     = 0; // volts (V)
    public static final double  kAlgaeArmkG                     = .4826; // volts (V)
    public static final double  kAlgaeArmKv                     = .02; // volts per velocity (V/RPM)
    public static final double  kAlgaeArmKa                     = 0; // volts per acceleration (V/(RPM/s))
    public static final double  kAlgaeAngleAllowableError       = 1;//degree, for testing whether it's aroundAngle

  }
  
  public static class CoralFunnelConstants {
    public static final int k_CoralFunnelWheelID = 15;
    public static final int k_CoralRotatorID = 14;
    
    public static final double k_CoralFunnelSpeed = -.15;
    public static final double k_CoralFunnelSpeedext = -.80;
    public static final double k_FunnelRotateSpeed = 0.1;
    
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
       
        public static final double   kElevatorGearing         = 12.0; // ours should be 5
        public static final double   kElevatorDrumDiameter      = Units.inchesToMeters(1.751);
       
        
        
        // Encoder is reset to measure 0 at the bottom, so minimum height is 0.
        public static final double   kMinElevatorHeightMeters = 0;//min height / 10
        public static final double   kMaxElevatorHeightMeters = 10.25;
        public static final Distance kMinElevatorHeight      = Meters.of(kMinElevatorHeightMeters);
        public static final Distance kMaxElevatorHeight      = Meters.of(kMaxElevatorHeightMeters);
        public static final double   kElevatorAllowableError = .04;
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


      }



    //Pose to align with Each Reef branch

      public static class TargetingConstants {
        public static final Pose2d ReefBranch_Red_A = new Pose2d(
          14.373098,
          4.163297232,
          Rotation2d.fromDegrees(0));

        public static final Pose2d ReefBranch_Red_AB_Middle = new Pose2d(
          14.360398,
          3.999029942,
          Rotation2d.fromDegrees(0));

        public static final Pose2d ReefBranch_Red_B = new Pose2d(
          13.58381383,
          2.766203768,
          Rotation2d.fromDegrees(0));

        public static final Pose2d ReefBranch_Red_C = new Pose2d(
          13.86845533,
          2.935113768,
          Rotation2d.fromDegrees(300));

        public static final Pose2d ReefBranch_Red_CD_Middle = new Pose2d(
          13.709396,
          2.872431894,
          Rotation2d.fromDegrees(300));

        public static final Pose2d ReefBranch_Red_D = new Pose2d(
          13.58381383,
          2.766203768,
          Rotation2d.fromDegrees(300));

        public static final Pose2d ReefBranch_Red_E = new Pose2d(
          12.48010863,
          2.766203768,
          Rotation2d.fromDegrees(240));

        public static final Pose2d ReefBranch_Red_EF_Middle = new Pose2d(
          12.408408,
          2.872431894,
          Rotation2d.fromDegrees(240));

        public static final Pose2d ReefBranch_Red_F = new Pose2d(
          12.19546715,
          2.935113768,
          Rotation2d.fromDegrees(240));

        public static final Pose2d ReefBranch_Red_G = new Pose2d(
          11.744706,
          3.888502768,
          Rotation2d.fromDegrees(180));

        public static final Pose2d ReefBranch_Red_GH_Middle = new Pose2d(
          11.757406,
          3.999029942,
          Rotation2d.fromDegrees(180));

        public static final Pose2d ReefBranch_Red_H = new Pose2d(
          11.744706,
          4.217178768,
          Rotation2d.fromDegrees(180));

        public static final Pose2d ReefBranch_Red_I = new Pose2d(
          12.24934867,
          5.116686232,
          Rotation2d.fromDegrees(120));

        public static final Pose2d ReefBranch_Red_IJ_Middle = new Pose2d(
          12.43534877,
          5.179368106,
          Rotation2d.fromDegrees(120));

        public static final Pose2d ReefBranch_Red_J = new Pose2d(
          12.53399017,
          5.285596232,
          Rotation2d.fromDegrees(120));

        public static final Pose2d ReefBranch_Red_K = new Pose2d(
          13.63769537,
          5.285596232,
          Rotation2d.fromDegrees(60));

        public static final Pose2d ReefBranch_Red_KL_Middle = new Pose2d(
          13.73626606,
          5.179368106,
          Rotation2d.fromDegrees(60));

        public static final Pose2d ReefBranch_Red_L = new Pose2d(
          13.92233687,
          5.116686232,
          Rotation2d.fromDegrees(60));





        public static final Pose2d ReefBranch_Blue_A = new Pose2d(
          3.175,
          3.888503068,
          Rotation2d.fromDegrees(0));

          public static final Pose2d ReefBranch_Blue_AB_Middle = new Pose2d(
            3.1877,
            3.999029942,
            Rotation2d.fromDegrees(0));

        public static final Pose2d ReefBranch_Blue_B = new Pose2d(
          3.175,
          4.217178768,
          Rotation2d.fromDegrees(0));

        public static final Pose2d ReefBranch_Blue_C = new Pose2d(
          3.679896628,
          5.116686232,
          Rotation2d.fromDegrees(300));

          public static final Pose2d ReefBranch_Blue_CD_Middle = new Pose2d(
            3.865826058,
            5.179368106,
            Rotation2d.fromDegrees(300));


        public static final Pose2d ReefBranch_Blue_D = new Pose2d(
          3.964538178,
          5.285596232,
          Rotation2d.fromDegrees(300));

        public static final Pose2d ReefBranch_Blue_E = new Pose2d(
          5.067989358,
          5.285596232,
          Rotation2d.fromDegrees(240));

          public static final Pose2d ReefBranch_Blue_EF_Middle = new Pose2d(
            5.166630768,
            5.179368106,
            Rotation2d.fromDegrees(240));


        public static final Pose2d ReefBranch_Blue_F = new Pose2d(
          5.352630908,
          5.116686,
          Rotation2d.fromDegrees(240));

        public static final Pose2d ReefBranch_Blue_G = new Pose2d(
          5.8036465,
          4.163297232,
          Rotation2d.fromDegrees(180));

          public static final Pose2d ReefBranch_Blue_GH_Middle = new Pose2d(
            5.790946,
            3.999029942,
            Rotation2d.fromDegrees(180));


        public static final Pose2d ReefBranch_Blue_H = new Pose2d(
          5.830586768,
          3.834621232,
          Rotation2d.fromDegrees(180));

        public static final Pose2d ReefBranch_Blue_I = new Pose2d(
          5.298749372,
          2.881232232,
          Rotation2d.fromDegrees(120));

          public static final Pose2d ReefBranch_Blue_IJ_Middle = new Pose2d(
            5.13969,
            2.872431894,
            Rotation2d.fromDegrees(120));


        public static final Pose2d ReefBranch_Blue_J = new Pose2d(
          5.014107822,
          2.766203768,
          Rotation2d.fromDegrees(120));

        public static final Pose2d ReefBranch_Blue_K = new Pose2d(
          3.910656642,
          2.766203768,
          Rotation2d.fromDegrees(60));

          public static final Pose2d ReefBranch_Blue_KL_Middle = new Pose2d(
            3.812015232,
            2.872431894,
            Rotation2d.fromDegrees(60));


        public static final Pose2d ReefBranch_Blue_L = new Pose2d(
          3.626015032,
          2.935113768,
          Rotation2d.fromDegrees(60));




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
