// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.SignalLights_commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import java.util.function.DoubleSupplier;

import frc.robot.subsystems.SignalLights_subsystem;

public class MakeLightsGo extends CommandBase {

  private final SignalLights_subsystem objLights;
  private DoubleSupplier dsSpeed;

  /** Creates a new MakeLightsGo. */
  public MakeLightsGo(SignalLights_subsystem objLights_in, DoubleSupplier dsSpeed_in) {
    objLights = objLights_in;
    dsSpeed = dsSpeed_in;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(objLights);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    objLights.chaseLights(dsSpeed.getAsDouble());
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    objLights.chaseLights(dsSpeed.getAsDouble());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    objLights.turnLightsOff();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
