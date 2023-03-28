// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Utilities;

import java.lang.Math;

public class Arm_subsystem extends SubsystemBase {

  private CANSparkMax objArmMotor1 = new CANSparkMax(Constants.canIDs.iArmMotor1, MotorType.kBrushless);
  private CANSparkMax objArmMotor2 = new CANSparkMax(Constants.canIDs.iArmMotor2, MotorType.kBrushless);
  private DutyCycleEncoder objAbsEncoder;
  private double dSpeed1;
  private double dSpeed2;
  private boolean bSoftStopActive;
  private boolean bSoftStopToHold;
  private boolean bHoldPosition;
  private double dHoldAngle = -99.0;
  private double dAngle;
  private double dCharSpeed;
  private boolean bRampStop;
  private boolean bArrived;

  /** Creates a new Arm_subsystem. */
  public Arm_subsystem() {
    objArmMotor1.setIdleMode(IdleMode.kBrake);
    objArmMotor1.setSmartCurrentLimit(Constants.Arm.iCurrentLimit);
    objArmMotor2.setIdleMode(IdleMode.kBrake);
    objArmMotor2.setSmartCurrentLimit(Constants.Arm.iCurrentLimit);
    objAbsEncoder = new DutyCycleEncoder(Constants.Arm.iDIOPort);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    dAngle = getArmAngle();
    if(bSoftStopActive) {
      softStop();
      if(Math.abs(objArmMotor1.get()) < 0.05) {
        bSoftStopActive = false;
        bHoldPosition = true;
        if (!bSoftStopToHold) dHoldAngle = dAngle;
      }
    }
    if (bHoldPosition) {
      holdPosition(dHoldAngle);
    }  

    if (Math.abs(dHoldAngle - dAngle) < Constants.Arm.dTolerance) bArrived = true;
    else bArrived = false;
    SmartDashboard.putBoolean("Arm Arrived", bArrived);
    SmartDashboard.putNumber("Arm Hold Angle", dHoldAngle);
  }

  public void setHoldAngle(double dHoldAngle_in) {
    dHoldAngle = dHoldAngle_in;
    bHoldPosition = true;
  }
  
  public void stopHoldingAngle() {
    bHoldPosition = false;
  }

  public void setSoftStop(boolean input) { 
    bSoftStopActive = input; 
    bSoftStopToHold = false;
    bHoldPosition = false;
  }

  public void setSoftStopToHold(double dHoldAngle_in) {
    bSoftStopActive = true;
    bSoftStopToHold = true;
    bHoldPosition = false;
    dHoldAngle = dHoldAngle_in;
  }

  public double moveArm(double dSpeed, double dSpeed_old) {
    double dSpeedLimit = Constants.Arm.dSpeedControlMax;
    double dCurrentAngle = getArmAngle();
    if (Math.abs(dSpeed) > Math.abs(dSpeed_old)) {      //Checking that speed is increasing
      dSpeed = dSpeed_old + Math.min(Math.abs(dSpeed - dSpeed_old), Constants.Arm.dSpeedUpLimit) * Math.signum(dSpeed);
    }

    if (dCurrentAngle > Constants.Arm.dMaxAngleLimit) {
      dSpeed = Utilities.limitVariable(-dSpeedLimit, dSpeed, 0.0);
    }
    else if (dCurrentAngle < Constants.Arm.dMinAngleLimit) {
      dSpeed = Utilities.limitVariable(0.0, dSpeed, dSpeedLimit);
    }
    objArmMotor1.set(dSpeed);
    objArmMotor2.set(-dSpeed);
    return dSpeed;
  }

  public void moveArm2(double dSpeed) {
    double dSpeedLimit = Constants.Arm.dSpeedControlMax;
    double dCurrentAngle = getArmAngle();
    if (dCurrentAngle > Constants.Arm.dMaxAngleLimit) {
      dSpeed = Utilities.limitVariable(-dSpeedLimit, dSpeed, 0.0);   
    }
    else if (dCurrentAngle < Constants.Arm.dMinAngleLimit) {
      dSpeed = Utilities.limitVariable(0.0, dSpeed, dSpeedLimit); 
    }
    // set speeds so positive speed moves in positive angle direction
    objArmMotor1.set(dSpeed);
    objArmMotor2.set(-dSpeed);
  }

  public double characterize(double dSpeed_in, double dMaxSpeed_in) {
    if (!bRampStop) {
      dCharSpeed = dSpeed_in + Constants.Arm.dRampLimit;
    }
    else {
      dCharSpeed = Math.max(dSpeed_in - Constants.Arm.dRampLimit, 0.0);
    }

    if (dCharSpeed >= dMaxSpeed_in) {
      bRampStop = true;
    }

    moveArm2(dCharSpeed);
    return dCharSpeed;
  }
  
  public void resetRamp() {
    bRampStop = false;
  }

  public void stopArm() {
    objArmMotor1.stopMotor();
    objArmMotor2.stopMotor();
  }

  public double softStop() {
    dSpeed1 = objArmMotor1.get();
    dSpeed2 = objArmMotor2.get();
    if (dSpeed1 > 0.0) {
      dSpeed1 = Math.max(dSpeed1 - Constants.Arm.dSoftStopLimit, 0.0);
      dSpeed2 = Math.min(dSpeed2 + Constants.Arm.dSoftStopLimit, 0.0);
    }
    else {
      dSpeed1 = Math.min(dSpeed1 + Constants.Arm.dSoftStopLimit, 0.0);
      dSpeed2 = Math.max(dSpeed2 - Constants.Arm.dSoftStopLimit, 0.0);
    }
    if(Math.abs(dSpeed1) < 0.035) {
      dSpeed1 = 0.0;
      dSpeed2 = 0.0;
    }
    objArmMotor1.set(dSpeed1);
    objArmMotor2.set(dSpeed2);
    return Math.abs(dSpeed1);
  }

  public double getArmAngle() {
    double dArmAngle;
    dArmAngle = Utilities.correctAngle2(objAbsEncoder.get(), Constants.Arm.dOffset, 1.0, false);

    SmartDashboard.putNumber("Raw Arm Encoder", objAbsEncoder.get());
    SmartDashboard.putNumber("Arm Angle", dArmAngle);
    
    return dArmAngle;
  }

  public double moveArmToAngle(double dTargetAngle, double dAngle_old, double dCommand_old, double dSpeedMult) {
    double dSpeedLimit = Constants.Arm.dSpeedControlMax;
    double dCurrentAngle = getArmAngle();
    double dDifference = dTargetAngle - dCurrentAngle; 
    double dDeriv;
    double dCommand;

    // computes dCommand, the motor speed
    dDeriv = dCurrentAngle - dAngle_old;
    if (Math.abs(dDifference) > 2.0) {
      dCommand = dDifference * Constants.Arm.kP - dDeriv * Constants.Arm.kD;
    }
    else {
      dCommand = dDifference * Constants.Arm.kP;
    }
    dCommand = Utilities.limitVariable(-dSpeedLimit * dSpeedMult, dCommand, dSpeedLimit * dSpeedMult);
    if (Math.abs(dCommand) > Math.abs(dCommand_old)) {      //Checking that speed is increasing
      dCommand = dCommand_old + Math.min(Math.abs(dCommand - dCommand_old), Constants.Arm.dSpeedUpLimit) * Math.signum(dCommand);
    }
    moveArm(dCommand, dCommand_old);
    return dCommand;
  }

  public double moveArmToAngle2(double dTargetAngle, double dCommand_old) {
    // note positive arm speed moves
    double dSpeedLimit = Constants.Arm.dSpeedControlMax;
    double dCurrentAngle = getArmAngle();
    double dDifference = dTargetAngle - dCurrentAngle; // To see if we have arrived
    double dCommand = 0.0;
    double dSign = Math.signum(dDifference);
    double dRampDownAngle;

    if (!bRampStop) {
      // if we haven't reached max speed, speed up to speed limit which is cruising speed
      dCommand = dCommand_old + dSign * Constants.Arm.dRampLimit;
    }
    dCommand = Utilities.limitVariable(-dSpeedLimit, dCommand, dSpeedLimit);
    dRampDownAngle = 0.5 * Constants.Arm.dRampCharKvalue * dCommand_old * dCommand_old / Constants.Arm.dRampLimit;
    if (Math.abs(dDifference) <= dRampDownAngle) {
      // if we are to the point where we need to slow down to arrive at the angle, set bRampStop to true to do the ramp down
      bRampStop = true;
    }
    // else bRampStop = false;

    if (bRampStop) {
      dCommand = dCommand_old - dSign * Constants.Arm.dRampLimit;
      if (dSign > 0.0) {
        // limit speed so do not ramp past zero when speed is positive
        dCommand = Math.max(dCommand, 0.0);
      }
      else {
        // limit speed so do not ramp past zero when speed is negative
        dCommand = Math.min(dCommand, 0.0);
      }
    }

    moveArm2(dCommand);
    return dCommand;
  }

  public void holdPosition(double dTargetAngle) {
    double dSpeedLimit = Constants.Arm.dSpeedControlMax;
    double dCurrentAngle = getArmAngle();
    double dDifference = dTargetAngle - dCurrentAngle;
    double dCommand = dDifference * 0.004;

    dCommand = Utilities.limitVariable(-dSpeedLimit, dCommand, dSpeedLimit);
    moveArm2(dCommand);
  }

}

