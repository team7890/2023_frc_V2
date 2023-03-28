// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

//Imports for Swerve Drive
import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
// import edu.wpi.first.math.trajectory.Trajectory;             //Unused
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import frc.lib.util.COTSFalconSwerveConstants;
import frc.lib.util.SwerveModuleConstants;
//End of Imports for Swerve

// imports for Limelight7028
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import static edu.wpi.first.math.util.Units.degreesToRadians;
// end of imports for Limelight

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

  public static final class canIDs{
    public static final int iArmMotor1 = 11;
    public static final int iArmMotor2 = 12;
    public static final int iForearmMotor = 13;
    public static final int iWristMotor = 10;
    public static final int iRollerMotor1 = 14;
    public static final int iRollerMotor2 = 15;
  }
  public static class Controllers {
    public static final int iDriver = 0;
    public static final int iCoPilot = 1;
    public static final int iButtonBox = 2;
    public static final int iButtonBoxV2 = 3;
  }

  public static final class Arm {
    // basic setup constants
    public static final int iCurrentLimit = 40;
    public static final int iDIOPort = 0;
    public static final double dOffset = 34.0;                         //Sets 0.0 to Straight up
    // main control constants
    public static final double dSpeedManual = 0.3;
    public static final double dMaxAngleLimit = 40.0;
    public static final double dMinAngleLimit = -40.0;
    public static final double dTolerance = 3.0;
    public static final double dSoftStopLimit = 0.035;
    // new ramp control constants
    public static final double dSpeedControlMax = 0.8;
    public static final double dRampLimit = 0.04;
    public static final double dRampCharKvalue = 1.83;
    // not really used anymore
    public static final double dDegreesPerRev = 360.0;
    public static final double kP = 0.5;
    public static final double kD = 0.5;
    public static final double dSpeedUpLimit = 0.005;    
  }

  public static final class Forearm {
    // basic setup constants
    public static final int iCurrentLimit = 40;
    public static final int iDIOPort = 1;
    public static final double dOffset = 148.5;                         //Sets 0.0 to Straight up
    // main control constants
    public static final double dSpeedManual = 0.15;
    public static final double dMaxAngleLimit = 160.0;
    public static final double dMinAngleLimit = -158.0;
    public static final double dTolerance = 3.0;
    public static final double dSoftStopLimit = 0.1;
    // new ramp control constants
    public static final double dSpeedControlMax = 0.6;
    public static final double dRampLimit = 0.03;
    public static final double dRampCharKvalue = 6.03;
    // not really used anymore
    public static final double dDegreesPerRev = 360.0;
    public static final double kP = 0.008;
    public static final double kD = 0.008;
    public static final double dSpeedUpLimit = 0.02;
  }

  public static final class Wrist{
    // basic setup constants
    public static final int iCurrentLimit = 40;
    public static final int iDIOPort = 2;
    public static final double dOffset = 123.1;               //Sets 0.0 to Straight up
    // main control constants
    public static final double dSpeedManual = 0.2;
    public static final double dMaxAngleLimit = 150.0;
    public static final double dMinAngleLimit = -150.0;
    public static final double dTolerance = 3.0;
    public static final double dSoftStopLimit = 0.2;
    // new ramp control constants
    public static final double dSpeedControlMax = 0.6;
    public static final double dRampLimit = 0.04;
    public static final double dRampCharKvalue = 11.37;    
    // not really used anymore
    public static final double dDegreesPerRev = 360.0 / 42.0 * 18.0;
    public static final double kP = 0.02;
    public static final double kD = 0.02;
    public static final double dSpeedUpLimit = 0.02;
  }

  public static final class Grabber{
    // Solenoids
    public static final int iChannel = 3;
  }




  
  //AUTO STUFF
  //Autonomous testingPath (PathPlanner)
  // public static final String testingPath = "output/TestingPath.wpilib.json";

  //Pathfinder Constant Class
  public static final class PathPlannerConstants {

      // Autonomous Period Constants
    public static final double autoMaxVelocity = 1.0; // meters/second
    public static final double autoMaxAcceleration = 0.5; // meters/second/second
    public static final double kPXController = 1.25;
    public static final double kPYController = 1.25;
    public static final double kPThetaController = 3.0;
    public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI / 2.0;
    public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI / 2.0;

    // Constraint for the motion profiled robot angle controller
    public static final TrapezoidProfile.Constraints kThetaControllerConstraints =
    new TrapezoidProfile.Constraints(kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);

    // Test Auto File Paths
    public static final String sTestPath = "Testing Path";
    public static final String sTestPath2 = "Smooth Side Pickup Red";
}




  //Beginning of Stuff Copied in for Swerve
  public static final double stickDeadband = 0.1;

    public static final class Swerve {
        public static final int pigeonID = 40;          //Changed Pidgeon ID to 40. This value was originally 0 and didn't say it had to be changed.
        public static final boolean invertGyro = false; // Always ensure Gyro is CCW+ CW-

        public static final COTSFalconSwerveConstants chosenModule =  //TODO: This must be tuned to specific robot
            COTSFalconSwerveConstants.SDSMK4i(COTSFalconSwerveConstants.driveGearRatios.SDSMK4i_L3);

        /* Drivetrain Constants */
        public static final double trackWidth = Units.inchesToMeters(20.75); //TODO: This must be tuned to specific robot
        public static final double wheelBase = Units.inchesToMeters(20.75); //TODO: This must be tuned to specific robot
        public static final double wheelCircumference = chosenModule.wheelCircumference;

        /* zero value constants to stop driving in FollowTrajectory */
        // public static final Trajectory zeroTrajectory = new Trajectory(null);
        public static final Translation2d zeroTranslation2d = new Translation2d();

        /* Swerve Kinematics 
         * No need to ever change this unless you are not doing a traditional rectangular/square 4 module swerve */
         public static final SwerveDriveKinematics swerveKinematics = new SwerveDriveKinematics(
            new Translation2d(wheelBase / 2.0, trackWidth / 2.0),
            new Translation2d(wheelBase / 2.0, -trackWidth / 2.0),
            new Translation2d(-wheelBase / 2.0, trackWidth / 2.0),
            new Translation2d(-wheelBase / 2.0, -trackWidth / 2.0));

        /* Module Gear Ratios */
        public static final double driveGearRatio = chosenModule.driveGearRatio;
        public static final double angleGearRatio = chosenModule.angleGearRatio;

        /* Motor Inverts */
        public static final boolean angleMotorInvert = chosenModule.angleMotorInvert;
        public static final boolean driveMotorInvert = chosenModule.driveMotorInvert;

        /* Angle Encoder Invert */
        public static final boolean canCoderInvert = chosenModule.canCoderInvert;

        /* Swerve Current Limiting */
        public static final int angleContinuousCurrentLimit = 25;
        public static final int anglePeakCurrentLimit = 30;                 // Changed from Team:364 default of 40
        public static final double anglePeakCurrentDuration = 0.1;
        public static final boolean angleEnableCurrentLimit = true;

        public static final int driveContinuousCurrentLimit = 35;
        public static final int drivePeakCurrentLimit = 40;                 // Changed from Team:364 default of 60
        public static final double drivePeakCurrentDuration = 0.1;
        public static final boolean driveEnableCurrentLimit = true;

        /* These values are used by the drive falcon to ramp in open loop and closed loop driving.
         * We found a small open loop ramp (0.25) helps with tread wear, tipping, etc */
        public static final double openLoopRamp = 0.25;
        public static final double closedLoopRamp = 0.0;

        /* Angle Motor PID Values */
        public static final double angleKP = chosenModule.angleKP;
        public static final double angleKI = chosenModule.angleKI;
        public static final double angleKD = chosenModule.angleKD;
        public static final double angleKF = chosenModule.angleKF;

        /* Drive Motor PID Values */
        public static final double driveKP = 0.05; //TODO: This must be tuned to specific robot
        public static final double driveKI = 0.0;
        public static final double driveKD = 0.0;
        public static final double driveKF = 0.0;

        /* Drive Motor Characterization Values 
         * Divide SYSID values by 12 to convert from volts to percent output for CTRE */
        public static final double driveKS = (0.32 / 12); //TODO: This must be tuned to specific robot
        public static final double driveKV = (1.51 / 12);
        public static final double driveKA = (0.27 / 12);

        /* Swerve Profiling Values */
        /** Meters per Second */
        public static final double maxSpeed = 4.5; //TODO: This must be tuned to specific robot
        /** Radians per Second */
        public static final double maxAngularVelocity = 10.0; //TODO: This must be tuned to specific robot

        /* Neutral Modes */
        public static final NeutralMode angleNeutralMode = NeutralMode.Coast;
        public static final NeutralMode driveNeutralMode = NeutralMode.Brake;

        /* Module Specific Constants */
        /* Front Left Module - Module 0 */
        public static final class Mod0 { //TODO: This must be tuned to specific robot
            public static final int driveMotorID = 23;
            public static final int angleMotorID = 22;
            public static final int canCoderID = 21;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(311.66);
            // public static final Rotation2d angleOffset = Rotation2d.fromDegrees(0.0);

            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }

        /* Front Right Module - Module 1 */
        public static final class Mod1 { //TODO: This must be tuned to specific robot
            public static final int driveMotorID = 26;
            public static final int angleMotorID = 25;
            public static final int canCoderID = 24;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(4.13);
            // public static final Rotation2d angleOffset = Rotation2d.fromDegrees(0.0);

            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }
        
        /* Back Left Module - Module 2 */
        public static final class Mod2 { //TODO: This must be tuned to specific robot
            public static final int driveMotorID = 33;
            public static final int angleMotorID = 32;
            public static final int canCoderID = 31;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(220.43);
            // public static final Rotation2d angleOffset = Rotation2d.fromDegrees(0.0);
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }

        /* Back Right Module - Module 3 */ 
        public static final class Mod3 { //TODO: This must be tuned to specific robot
            public static final int driveMotorID = 36;
            public static final int angleMotorID = 35;
            public static final int canCoderID = 34;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(305.51);   
            // public static final Rotation2d angleOffset = Rotation2d.fromDegrees(0.0);
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }
    }

    public static final class AutoConstants { //TODO: The below constants are used in the example auto, and must be tuned to specific robot
        public static final double kMaxSpeedMetersPerSecond = 3;
        public static final double kMaxAccelerationMetersPerSecondSquared = 3;
        public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
        public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;
    
        public static final double kPXController = 1;
        public static final double kPYController = 1;
        public static final double kPThetaController = 1;
    
        /* Constraint for the motion profilied robot angle controller */
        public static final TrapezoidProfile.Constraints kThetaControllerConstraints =
            new TrapezoidProfile.Constraints(
                kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
    }
  //End of Stuff Copied in for Swerve

  // stuff copied in for Limelight7028
  public static class VisionConstants {

    /** Physical location of the apriltag camera on the robot, relative to the center of the robot. */
    public static final Transform3d APRILTAG_CAMERA_TO_ROBOT =
        new Transform3d(new Translation3d(-0.06, 0.250, -0.2127), new Rotation3d(0.0, degreesToRadians(15.0), 0.0));

    /** Physical location of the shooter camera on the robot, relative to the center of the robot. */
    public static final Transform3d LOW_LIMELIGHT_TO_ROBOT = new Transform3d(
        new Translation3d(-0.083, 0.254, -0.537),
        new Rotation3d(0.0, degreesToRadians(-9.8), degreesToRadians(-1.0)));

    public static final String LOW_LIMELIGHT_NAME = "limelight";
    
    /** Physical location of the high camera on the robot, relative to the center of the robot. */
    public static final Transform3d HIGH_LIMELIGHT_TO_ROBOT = new Transform3d(
        new Translation3d(-0.11, -0.015, -0.895),
        new Rotation3d(degreesToRadians(-90.0), degreesToRadians(34.6), 0.0));

    public static final String HIGH_LIMELIGHT_NAME = "limelight-high";
    
    public static final double FIELD_LENGTH_METERS = 16.54175;
    public static final double FIELD_WIDTH_METERS = 8.0137;

    // Pose on the opposite side of the field. Use with `relativeTo` to flip a pose to the opposite alliance
    public static final Pose2d FLIPPING_POSE = new Pose2d(
        new Translation2d(FIELD_LENGTH_METERS, FIELD_WIDTH_METERS),
        new Rotation2d(Math.PI));

    /** Minimum target ambiguity. Targets with higher ambiguity will be discarded */
    public static final double APRILTAG_AMBIGUITY_THRESHOLD = 0.2;
  }
  // end of stuff copied in for Limelight


}
