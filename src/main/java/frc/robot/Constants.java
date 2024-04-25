package frc.robot;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import frc.lib.util.COTSTalonFXSwerveConstants;
import frc.lib.util.SwerveModuleConstants;

public final class Constants {
    public static final double stickDeadband = 0.1;

    public static final class Swerve {
        public static final int pigeonID = 9;
        public static final String canivorename = "canivore";

        public static final COTSTalonFXSwerveConstants chosenModule =  
        COTSTalonFXSwerveConstants.SDS.MK4i.Falcon500(COTSTalonFXSwerveConstants.SDS.MK4i.driveRatios.L3);

        /* Drivetrain Constants */
        public static final double trackWidth = Units.inchesToMeters(20);
        public static final double wheelBase = Units.inchesToMeters(22.75);
        public static final double wheelCircumference = chosenModule.wheelCircumference;

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
        public static final InvertedValue angleMotorInvert = chosenModule.angleMotorInvert;
        public static final InvertedValue driveMotorInvert = chosenModule.driveMotorInvert;

        /* Angle Encoder Invert */
        public static final SensorDirectionValue cancoderInvert = chosenModule.cancoderInvert;

        /* Swerve Current Limiting */
        public static final int angleCurrentLimit = 25;
        public static final int angleCurrentThreshold = 40;
        public static final double angleCurrentThresholdTime = 0.1;
        public static final boolean angleEnableCurrentLimit = true;

        public static final int driveCurrentLimit = 35;
        public static final int driveCurrentThreshold = 60;
        public static final double driveCurrentThresholdTime = 0.1;
        public static final boolean driveEnableCurrentLimit = true;

        /* These values are used by the drive falcon to ramp in open loop and closed loop driving.
         * We found a small open loop ramp (0.25) helps with tread wear, tipping, etc */
        public static final double openLoopRamp = 0.25;
        public static final double closedLoopRamp = 0.0;

        /* Angle Motor PID Values */
        public static final double angleKP = chosenModule.angleKP;
        public static final double angleKI = chosenModule.angleKI;
        public static final double angleKD = chosenModule.angleKD;

        /* Drive Motor PID Values */
        public static final double driveKP = 0.12;
        public static final double driveKI = 0.0;
        public static final double driveKD = 0.0;
        public static final double driveKF = 0.0;

        /* Drive Motor Characterization Values From SYSID */
        public static final double driveKS = 0.32; 
        public static final double driveKV = 1.51;
        public static final double driveKA = 0.27;

        /* Swerve Profiling Values */
        /** Meters per Second */
        public static final double maxSpeed = 5.6;
        /** Radians per Second */
        public static final double maxAngularVelocity = 10.0;

        /* Neutral Modes */
        public static final NeutralModeValue angleNeutralMode = NeutralModeValue.Coast;
        public static final NeutralModeValue driveNeutralMode = NeutralModeValue.Brake;

        /* Module Specific Constants */
        /* Front Right Module - Module 0 */
        public static final class Mod0 { 
            public static final int driveMotorID = 3;
            public static final int angleMotorID = 4;
            public static final int canCoderID = 1;
            public static final Rotation2d angleOffset = Rotation2d.fromRotations(-0.326904296875+.5);
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }

        /* Front Left Module - Module 1 */
        public static final class Mod1 { 
            public static final int driveMotorID = 1;
            public static final int angleMotorID = 2;
            public static final int canCoderID = 2;
            public static final Rotation2d angleOffset = Rotation2d.fromRotations(-0.116943359375);
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }
        
        /* Back Right Module - Module 2 */
        public static final class Mod2 { 
            public static final int driveMotorID = 5;
            public static final int angleMotorID = 6;
            public static final int canCoderID = 3;
            public static final Rotation2d angleOffset = Rotation2d.fromRotations(-0.05712890625+.5);
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }

        /* Back Left Module - Module 3 */
        public static final class Mod3 { 
            public static final int driveMotorID = 7;
            public static final int angleMotorID = 8;
            public static final int canCoderID = 4;
            public static final Rotation2d angleOffset = Rotation2d.fromRotations(0.39404296875);
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }
    }

    public static final class AutoConstants { 
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

    public static final class ShooterConstants{
        public static int shooterMasterID = 15;
        public static int shooterFollowerID = 16;
        public static int conveyorID = 10;
        public static int beamBreakChannelID = 3;
        public static int conveyorIntakeOutput = 12;
        public static int conveyorOuttakeOutput = -5;
        public static double shooterPodiumVelocity = 12;
        public static double shooterSpeakerVelocity = 9;
        public static double shooterLowerVelocity = 5;
    }


    public static final class PivotConstants{
        public static int pivotMasterID = 11;
        public static int pivotFollowerID = 13;
        public static int pivotCANCoderID = 5;
        public static double pivotKG = 0.2;
        public static double pivotKP = 5;
        public static double pivotKD = 0;
        public static double pivotKV = 0.0;
        public static double ampSetPoint = 8.068;
    }

    public static final class ClimberConstants{
        public static int climberID = 14;
    }

    public static final class IntakeConstants{
        public static int intakeMasterID = 9;
        public static int intakeFollowerID = 12;
        public static int intakeOutput = 10;
        public static int outtakeOutput = -5;
    }

    public static record pivotState(double pivotPos){}
    public static record shooterState(double shooterVolt){}

    // key is distance (TY), value is pivot position (Rotations)
    public static final InterpolatingDoubleTreeMap PIVOT_STATE_MAP = new InterpolatingDoubleTreeMap();
    static{
        PIVOT_STATE_MAP.put(0.0, 0.0);
        PIVOT_STATE_MAP.put(-3.4, 0.76);
        PIVOT_STATE_MAP.put(-6.0, 0.85);
        PIVOT_STATE_MAP.put(-9.18, 1.1);
        PIVOT_STATE_MAP.put(-12.0, 1.2);
        PIVOT_STATE_MAP.put(-13.18, 1.31);
        PIVOT_STATE_MAP.put(-15.0, 1.325);
        PIVOT_STATE_MAP.put(-16.0, 1.6);
        PIVOT_STATE_MAP.put(-16.64, 1.65);
        PIVOT_STATE_MAP.put(-18.5, 1.7);
        PIVOT_STATE_MAP.put(-19.5, 2.0);
        PIVOT_STATE_MAP.put(-20.5, 2.075);
        PIVOT_STATE_MAP.put(-22.0, 2.15);
        PIVOT_STATE_MAP.put(-23.25, 2.2);
        PIVOT_STATE_MAP.put(-25.0, 2.3);
        PIVOT_STATE_MAP.put(-26.0, 2.41);
        PIVOT_STATE_MAP.put(-28.0, 2.48);
        PIVOT_STATE_MAP.put(-28.8, 2.50);
        
    }

    // key is distance (TY), value is shooter RPMS (Voltage input)
    public static final InterpolatingDoubleTreeMap VELOCITY_STATE_MAP = new InterpolatingDoubleTreeMap();
    static{
        VELOCITY_STATE_MAP.put(0.0, 6.5);
        VELOCITY_STATE_MAP.put(-3.4, 6.6);
        VELOCITY_STATE_MAP.put(-6.0, 6.7);
        VELOCITY_STATE_MAP.put(-9.18, 6.75);
        VELOCITY_STATE_MAP.put(-12.0, 7.75);
        VELOCITY_STATE_MAP.put(-13.18, 7.8);
        VELOCITY_STATE_MAP.put(-16.0, 7.8);
        VELOCITY_STATE_MAP.put(-18.5, 8.0);
        VELOCITY_STATE_MAP.put(-20.5, 8.75);
        VELOCITY_STATE_MAP.put(-23.25, 9.9);
        VELOCITY_STATE_MAP.put(-28.0, 10.5);
    }
       

}    

