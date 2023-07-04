// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;

public class RollerHand_subsystem extends SubsystemBase {

  private CANSparkMax objMotor1 = new CANSparkMax(Constants.canIDs.iRollerMotor1, MotorType.kBrushless); 
  private CANSparkMax objMotor2 = new CANSparkMax(Constants.canIDs.iRollerMotor2, MotorType.kBrushless);

  /** Creates a new Grabber. */
  public RollerHand_subsystem() {
    objMotor1.setIdleMode(IdleMode.kBrake);
    objMotor2.setIdleMode(IdleMode.kBrake);
    objMotor1.setSmartCurrentLimit(35); // TODO: Make constants
    objMotor2.setSmartCurrentLimit(35);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    // SmartDashboard.putNumber("Motor 1 Current", getMotor1Current());
  }

  public void intakeCone() {
    double dSpeed = 0.5;
    objMotor1.set(dSpeed);
    objMotor2.set(dSpeed);
  }

  public void intakeCone_SingleSubstation() {
    double dSpeed = 0.2;
    objMotor1.set(dSpeed);
    objMotor2.set(dSpeed);
  }

  public void holdCone() {
    double dSpeed = 0.025;
    objMotor1.set(dSpeed);
    objMotor2.set(dSpeed);
  }

  public void outtakeCone() {
    double dSpeed = 0.5;
    objMotor1.set(-dSpeed);
    objMotor2.set(-dSpeed);
    // return moveRollers(-dIntakeSpeed, dSpeed_old, false);
  }

  public void intakeCube() {
    double dSpeed = 0.75;
    objMotor1.set(-dSpeed);
    objMotor2.set(-dSpeed);
  }

  public void holdCube() {
    double dSpeed = 0.03;
    objMotor1.set(-dSpeed);
    objMotor2.set(-dSpeed);
  }

  public void outtakeCube(boolean bDirection) {
    // bDirection is true for out the top and false for out the bottom (go out same way it came in when intaking)
    double dSpeed = 0.5;
    if (bDirection) {
      objMotor1.set(dSpeed);
      objMotor2.set(-dSpeed);
    }
    else {
      objMotor1.set(-dSpeed);
      objMotor2.set(dSpeed);
    }
  }

  public double getMotor1Current() {
    return objMotor1.getOutputCurrent();
  }

  public double getSpeed() {
    return objMotor1.get();
  }

  public void stopMotors() {
    objMotor1.stopMotor();
    objMotor2.stopMotor();
  }
}
