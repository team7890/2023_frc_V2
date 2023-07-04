package frc.robot.subsystems;

import frc.robot.SwerveModule;
import frc.robot.Constants;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;

import java.net.PortUnreachableException;

import com.ctre.phoenix.sensors.Pigeon2;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

// ShuffleBoard imports
// import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
// import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

public class Swerve_subsystem extends SubsystemBase {
    public SwerveDriveOdometry swerveOdometry;
    public SwerveModule[] mSwerveMods;
    public Pigeon2 gyro;

    // debug
    public SwerveModuleState sModState;

    public Swerve_subsystem() {
        gyro = new Pigeon2(Constants.Swerve.pigeonID, "rio");       // Added "rio" network (bc it's wired on rio/pdh network)
        gyro.configFactoryDefault();
        flipGyro();
        // zeroGyro();

        mSwerveMods = new SwerveModule[] {
            new SwerveModule(0, Constants.Swerve.Mod0.constants),
            new SwerveModule(1, Constants.Swerve.Mod1.constants),
            new SwerveModule(2, Constants.Swerve.Mod2.constants),
            new SwerveModule(3, Constants.Swerve.Mod3.constants)
        };

        /* By pausing init for a second before setting module offsets, we avoid a bug with inverting motors.
         * See https://github.com/Team364/BaseFalconSwerve/issues/8 for more info.
         */
        Timer.delay(1.0);
        resetModulesToAbsolute();

        swerveOdometry = new SwerveDriveOdometry(Constants.Swerve.swerveKinematics, getYaw(), getModulePositions());
    }

    public void drive(Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop) {
        SwerveModuleState[] swerveModuleStates =
            Constants.Swerve.swerveKinematics.toSwerveModuleStates(
                fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(
                                    translation.getX(), 
                                    translation.getY(), 
                                    rotation, 
                                    getYaw()
                                )
                                : new ChassisSpeeds(
                                    translation.getX(), 
                                    translation.getY(), 
                                    rotation)
                                );
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.Swerve.maxSpeed);

        for(SwerveModule mod : mSwerveMods){
            mod.setDesiredState(swerveModuleStates[mod.moduleNumber], isOpenLoop);
            // // debug swerve angle change
            // if (mod.moduleNumber == 1) {
            //     sModState = swerveModuleStates[mod.moduleNumber];
            //     double dRot = sModState.angle.getDegrees();
            //     SmartDashboard.putNumber("Mod 1 State Rot", dRot);
            // }
            // // end debug
        }
    }    

    /* Used by SwerveControllerCommand in Auto */
    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.Swerve.maxSpeed);
        
        for(SwerveModule mod : mSwerveMods){
            mod.setDesiredState(desiredStates[mod.moduleNumber], false);
        }
    }    

    public Pose2d getPose() {
        return swerveOdometry.getPoseMeters();
    }

    public void resetOdometry(Pose2d pose) {
        // swerveOdometry.resetPosition(getYaw(), getModulePositions(), pose);
        swerveOdometry.resetPosition(getYawFixed(), getModulePositions(), pose);
    }

    public SwerveModuleState[] getModuleStates(){
        SwerveModuleState[] states = new SwerveModuleState[4];
        for(SwerveModule mod : mSwerveMods){
            states[mod.moduleNumber] = mod.getState();
        }
        return states;
    }

    public SwerveModulePosition[] getModulePositions(){
        SwerveModulePosition[] positions = new SwerveModulePosition[4];
        for(SwerveModule mod : mSwerveMods){
            positions[mod.moduleNumber] = mod.getPosition();
        }
        return positions;
    }

    public void zeroGyro(){
        gyro.setYaw(0);
    }

    public void flipGyro(){
        gyro.setYaw(180);
    }

    public Rotation2d getYaw() {
        return (Constants.Swerve.invertGyro) ? Rotation2d.fromDegrees(360 - gyro.getYaw()) : Rotation2d.fromDegrees(gyro.getYaw());
    }

    public Rotation2d getYawFlipped() {
        double dGyroYaw = gyro.getYaw() + 180.0;
        return (Constants.Swerve.invertGyro) ? Rotation2d.fromDegrees(360 - dGyroYaw) : Rotation2d.fromDegrees(dGyroYaw);
    }

    public Rotation2d getYawFixed() {
        double dGyroYaw = gyro.getYaw(); // + 5.0;
        return (Constants.Swerve.invertGyro) ? Rotation2d.fromDegrees(360 - dGyroYaw) : Rotation2d.fromDegrees(dGyroYaw);
    }

    public void resetModulesToAbsolute(){
        for(SwerveModule mod : mSwerveMods){
            mod.resetToAbsolute();
        }
    }

    public double getPitch() {
        double dPitch = gyro.getPitch();
        return dPitch;
    }

    public double getRoll() {
        double dRoll = gyro.getRoll();
        return dRoll;
    }

    public double getYawDouble() {
        double dYaw = gyro.getYaw() % 360.0;
        if (dYaw < 0.0) dYaw = dYaw + 360.0;
        return dYaw;
    }

    @Override
    public void periodic(){
        swerveOdometry.update(getYaw(), getModulePositions()); 
        // for(SwerveModule mod : mSwerveMods){
        //     // debug swerve angle change
        //     if (mod.moduleNumber == 1) {
        //         Rotation2d r2d = mod.getState().angle;
        //         SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Angle", r2d.getDegrees());
        //     }
        //     // end debug

        //     // SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Cancoder", mod.getCanCoder().getDegrees());
        //     // SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Integrated", mod.getDistanceMeters());
        //     // SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Velocity", mod.getState().speedMetersPerSecond); 
        // }
        // SmartDashboard.putNumber("Roll: ", getRoll());
        // SmartDashboard.putNumber("Yaw", getYawDouble());

        // SmartDashboard.putNumber("mod1 angle", mod1.getAngle())
    }
}