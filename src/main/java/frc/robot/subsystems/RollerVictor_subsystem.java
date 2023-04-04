// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.VictorSPXControlMode;

// import com.revrobotics.CANSparkMax;
// import com.revrobotics.CANSparkMax.IdleMode;
// import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;

public class RollerVictor_subsystem extends SubsystemBase {

  // private CANSparkMax objMotor1 = new CANSparkMax(Constants.canIDs.iRollerMotor1, MotorType.kBrushless); 
  // private CANSparkMax objMotor2 = new CANSparkMax(Constants.canIDs.iRollerMotor2, MotorType.kBrushless);
  
  // TODO: Make constants
  private VictorSPX objMotor1 = new VictorSPX(16);
  private VictorSPX objMotor2 = new VictorSPX(17);

  /** Creates a new Grabber. */
  public RollerVictor_subsystem() {
    objMotor1.setNeutralMode(NeutralMode.Brake);
    objMotor2.setNeutralMode(NeutralMode.Brake);
    // objMotor1.setInverted(true);
    // objMotor2.setInverted(true);
    
    // objMotor1.setSmartCurrentLimit(3); // TODO: Make constants
    // objMotor2.setSmartCurrentLimit(3);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    // SmartDashboard.putNumber("Motor 1 Current", getMotor1Current());
  }

  public void intakeCone() {
    double dSpeed = 0.5;
    objMotor1.set(VictorSPXControlMode.PercentOutput, dSpeed);
    objMotor2.set(VictorSPXControlMode.PercentOutput, dSpeed);
  }

  public void intakeCone_SingleSubstation() {
    double dSpeed = 0.2;
    objMotor1.set(VictorSPXControlMode.PercentOutput, dSpeed);
    objMotor2.set(VictorSPXControlMode.PercentOutput, dSpeed);
  }

  public void holdCone() {
    double dSpeed = 0.025;
    objMotor1.set(VictorSPXControlMode.PercentOutput, dSpeed);
    objMotor2.set(VictorSPXControlMode.PercentOutput, dSpeed);
  }

  public void outtakeCone() {
    double dSpeed = 0.5;
    objMotor1.set(VictorSPXControlMode.PercentOutput, -dSpeed);
    objMotor2.set(VictorSPXControlMode.PercentOutput, -dSpeed);
    // return moveRollers(-dIntakeSpeed, dSpeed_old, false);
  }

  public void intakeCube() {
    double dSpeed = 0.75;
    objMotor1.set(VictorSPXControlMode.PercentOutput, -dSpeed);
    objMotor2.set(VictorSPXControlMode.PercentOutput, -dSpeed);
  }

  public void outtakeCube(boolean bDirection) {
    // bDirection is true for out the top and false for out the bottom (go out same way it came in when intaking)
    double dSpeed = 0.5;
    if (bDirection) {
      objMotor1.set(VictorSPXControlMode.PercentOutput, dSpeed);
      objMotor2.set(VictorSPXControlMode.PercentOutput, -dSpeed);
    }
    else {
      objMotor1.set(VictorSPXControlMode.PercentOutput, -dSpeed);
      objMotor2.set(VictorSPXControlMode.PercentOutput, dSpeed);
    }
  }

  // public double getMotor1Current() {
  //   return objMotor1.get();
  // }

  // public double getSpeed() {
  //   return objMotor1.get();
  // }

  public void stopMotors() {
    objMotor1.set(VictorSPXControlMode.PercentOutput, 0.0);
    objMotor2.set(VictorSPXControlMode.PercentOutput, 0.0);
  }
}
