// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import frc.robot.Utilities;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.lang.Math;

public class Forearm_subsystem extends SubsystemBase {

  private CANSparkMax objForearmMotor = new CANSparkMax(Constants.canIDs.iForearmMotor,MotorType.kBrushless);
  private DutyCycleEncoder objAbsEncoder;
  private double dSpeed;
  private boolean bSoftStopActive;
  private boolean bHoldPosition;
  private double dHoldAngle;
  private double dAngle;
  private double dCharSpeed;
  private boolean bRampStop;

  /** Creates a new Forearm_subsystem. */
  public Forearm_subsystem() {
    objForearmMotor.setIdleMode(IdleMode.kBrake);
    objForearmMotor.setSmartCurrentLimit(Constants.Forearm.iCurrentLimit);
    objAbsEncoder = new DutyCycleEncoder(Constants.Forearm.iDIOPort);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    dAngle = getForearmAngle();
    // if (bSoftStopActive) {
    //   softStop();
    //   if (Math.abs(objForearmMotor.get()) < 0.03) {
    //     bSoftStopActive = false;
    //     bHoldPosition = true;
    //     dHoldAngle = dAngle;
    //   }
    // }

    if (bHoldPosition) {
      holdPosition(dHoldAngle, 1.0);
    }    
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
    bHoldPosition = false;
  }

  public void moveForearm(double dSpeed) {
    double dSpeedLimit = Constants.Forearm.dSpeedControlMax;
    double dCurrentAngle = getForearmAngle();
    if (dCurrentAngle > Constants.Forearm.dMaxAngleLimit) {
      dSpeed = Utilities.limitVariable(-dSpeedLimit, dSpeed, 0.0);   
    }
    else if (dCurrentAngle < Constants.Forearm.dMinAngleLimit) {
      dSpeed = Utilities.limitVariable(0.0, dSpeed, dSpeedLimit); 
    }
    objForearmMotor.set(dSpeed);
  }

  public double characterize(double dSpeed_in, double dMaxSpeed_in) {
    if (!bRampStop) {
      dCharSpeed = dSpeed_in + Constants.Forearm.dRampLimit;
    }
    else {
      dCharSpeed = Math.max(dSpeed_in - Constants.Forearm.dRampLimit, 0.0);
    }

    if (dCharSpeed >= dMaxSpeed_in) {
      bRampStop = true;
    }

    moveForearm(dCharSpeed);
    return dCharSpeed;
  }
  

  public void resetRamp() {
    bRampStop = false;
  }

  public void stopForearm() {
    objForearmMotor.stopMotor();
  }

  public double softStop() {
    dSpeed = objForearmMotor.get();
    if (dSpeed > 0.0) {
      dSpeed = Math.max(dSpeed - Constants.Forearm.dSoftStopLimit, 0.0);
    }
    else {
      dSpeed = Math.min(dSpeed + Constants.Forearm.dSoftStopLimit, 0.0);
    }
    objForearmMotor.set(dSpeed);
    return Math.abs(dSpeed);
  }

  public double getForearmAngle() {
    double dForearmAngle;
    dForearmAngle = Utilities.correctAngle2(objAbsEncoder.get(), Constants.Forearm.dOffset, 1.0, false);

    SmartDashboard.putNumber("Raw Forearm Encoder", objAbsEncoder.get());
    SmartDashboard.putNumber("Forearm Angle", dForearmAngle);
    
    return dForearmAngle;
  }

  public double moveForearmToAngle(double dTargetAngle, double dAngle_old, double dCommand_old, double dSpeedMult) {
    double dSpeedLimit = Constants.Forearm.dSpeedControlMax;
    double dCurrentAngle = getForearmAngle();
    double dDifference = dTargetAngle - dCurrentAngle; 
    double dDeriv;
    boolean bArrived = false;
    double dCommand;

    // computes dCommand, the motor speed
    dDeriv = dCurrentAngle - dAngle_old;
    if (Math.abs(dDifference) > 2.0) {
      dCommand = dDifference * Constants.Forearm.kP - dDeriv * Constants.Forearm.kD;
    }
    else {
      dCommand = dDifference * Constants.Forearm.kP;
    }

    dCommand = Utilities.limitVariable(-dSpeedLimit * dSpeedMult, dCommand, dSpeedLimit * dSpeedMult);
    if (Math.abs(dCommand) > Math.abs(dCommand_old)) {      //Checking that speed is increasing
      dCommand = dCommand_old + Math.min(Math.abs(dCommand - dCommand_old), Constants.Forearm.dSpeedUpLimit) * Math.signum(dCommand);
    }
    moveForearm(dCommand);
    if (Math.abs(dDifference) < Constants.Forearm.dTolerance) {
      bArrived = true;
    }
    SmartDashboard.putBoolean("Forearm Arrived", bArrived);
    return dCommand;
  }
  
  public double moveForearmToAngle2(double dTargetAngle, double dCommand_old) {
    double dSpeedLimit = Constants.Forearm.dSpeedControlMax;
    double dCurrentAngle = getForearmAngle();
    double dDifference = dTargetAngle - dCurrentAngle; // To see if we have arrived
    boolean bArrived = false;
    double dCommand = 0.0;
    double dSign = Math.signum(dDifference);

    if (!bRampStop) {
      // if we haven't reached max speed, speed up to speed limit which is cruising speed
      dCommand = dCommand_old + dSign * Constants.Forearm.dRampLimit;
    }
    dCommand = Utilities.limitVariable(-dSpeedLimit, dCommand, dSpeedLimit);
    if (Math.abs(dDifference) <= 20.5) {
      // if we are to the point where we need to slow down to arrive at the angle, set bRampStop to true to do the ramp down
      bRampStop = true;
    }
    if (bRampStop) {
      dCommand = dCommand_old - dSign * Constants.Forearm.dRampLimit;
      if (dSign > 0.0) {
        // limit speed so do not ramp past zero when speed is positive
        dCommand = Math.max(dCommand, 0.0);
      }
      else {
        // limit speed so do not ramp past zero when speed is negative
        dCommand = Math.min(dCommand, 0.0);
      }
    }

    moveForearm(dCommand);
    if (Math.abs(dDifference) < Constants.Forearm.dTolerance || dCommand == 0.0) {
      bArrived = true;
    }
    SmartDashboard.putBoolean("Forearm Arrived", bArrived);
    return dCommand;
  }

  public void holdPosition(double dTargetAngle, double dSpeedMult) {
    double dSpeedLimit = Constants.Forearm.dSpeedControlMax;
    double dCurrentAngle = getForearmAngle();
    double dDifference = dTargetAngle - dCurrentAngle; 
    double dCommand = dDifference * (Constants.Forearm.kP * 0.6);

    if (dCurrentAngle > 25.0) {
      dCommand = dCommand - 0.025;
    }
    if (dCurrentAngle < -25.0) {
      dCommand = dCommand + 0.025;
    }

    dCommand = Utilities.limitVariable(-dSpeedLimit * dSpeedMult, dCommand, dSpeedLimit * dSpeedMult);
    moveForearm(dCommand);
  }

}