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
    if (bSoftStopActive) {
      softStop();
      if (Math.abs(objForearmMotor.get()) < 0.03) {
        bSoftStopActive = false;
        bHoldPosition = true;
        dHoldAngle = dAngle;
      }
    }

    if(bHoldPosition) {
      holdPosition(dHoldAngle, 1.0);
    }    
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
    if(Math.abs(dSpeed) < 0.06) { // former was < .035, this used to not be herei
      dSpeed = 0.0;
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
      // if (!(getForearmAngle() > 20.0 && dCommand > 0.0))
      // if (!(getForearmAngle() < -20.0 && dCommand < 0.0))
      dCommand = dCommand + Math.signum(dCommand) * 0.05;
    }
    else {
      dCommand = dDifference * Constants.Forearm.kP;
    }
    // if(Math.abs(dDifference) < 0.75) dCommand = 0.0;

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

  public void holdPosition(double dTargetAngle, double dSpeedMult) {
    double dSpeedLimit = Constants.Forearm.dSpeedControlMax;
    double dCurrentAngle = getForearmAngle();
    double dDifference = dTargetAngle - dCurrentAngle; 
    // double dDeriv;
    // boolean bArrived = false;

    // computes dCommand, the motor speed
    // dDeriv = dCurrentAngle - dAngle_old;
    // double dCommand = dDifference * Constants.Forearm.kP - dDeriv * Constants.Forearm.kD;
    // if(Math.abs(dDifference) < 0.75) dCommand = 0.0;

    double dCommand = dDifference * (Constants.Forearm.kP * 0.6);
    if (dCurrentAngle > 25.0) {
      dCommand = dCommand - 0.025;
    }
    if (dCurrentAngle < -25.0) {
      dCommand = dCommand + 0.025;
    }

    dCommand = Utilities.limitVariable(-dSpeedLimit * dSpeedMult, dCommand, dSpeedLimit * dSpeedMult);
    // if (Math.abs(dCommand) > Math.abs(dCommand_old)) {      //Checking that speed is increasing
    //   dCommand = dCommand_old + Math.min(Math.abs(dCommand - dCommand_old), Constants.Forearm.dSpeedUpLimit) * Math.signum(dCommand);
    // }
    moveForearm(dCommand);
    // if (Math.abs(dDifference) < Constants.Forearm.dTolerance) {
    //   bArrived = true;
    // }
    // return dCommand;
  }

}