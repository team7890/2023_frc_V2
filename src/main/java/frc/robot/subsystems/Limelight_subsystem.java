// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

// import frc.robot.LimelightHelpers;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

public class Limelight_subsystem extends SubsystemBase {

  NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
  NetworkTableEntry tx = table.getEntry("tx");
  NetworkTableEntry ty = table.getEntry("ty");
  NetworkTableEntry ta = table.getEntry("ta");
  NetworkTableEntry ts = table.getEntry("ts");
  NetworkTableEntry tv = table.getEntry("tv");

  /** Creates a new Limelight_subsystem. */
  public Limelight_subsystem() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    // LimelightHelpers.LimelightResults objLimelightResults = LimelightHelpers.getLatestResults("limelight");
    // var vResults = objLimelightResults.targetingResults.targets_Fiducials[0];
    // tx = vResults.tx;
    // ty = vResults.ty;

    SmartDashboard.putNumber("tx", tx.getDouble(0.0));
    SmartDashboard.putNumber("ty", ty.getDouble(0.0));
    SmartDashboard.putNumber("ta", ta.getDouble(0.0));
    SmartDashboard.putNumber("ts", ts.getDouble(0.0));
    SmartDashboard.putBoolean("tv", tv.getBoolean(false));
  }

  public void turnLEDsOn() {
    // ledMode 3 is force on
    NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(3);
  }

  public void turnLEDsOff() {
    // ledMode 1 is force off
    NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(1);
  }

  public void setLEDsPerPipeline() {
    // ledMode 0 is use mode in current pipeline
    NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(0);
  }
  
  public void setPipeline(int iNum) {
    NetworkTableInstance.getDefault().getTable("limelight").getEntry("pipeline").setNumber(0);
  }

  public double getAngle() {
    return tx.getDouble(0.0);
  }

  public boolean isTargetAcquired() {
    return tv.getBoolean(false);
  }

}
