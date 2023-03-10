// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

// Our Imports
import frc.robot.Constants;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.PneumaticHub;

public class xGrabber_subsystem extends SubsystemBase {
  /** Creates a new Grabber. */
  
  private Compressor objCompressor;
  private Solenoid objSolenoid;
  private boolean bState;

  public xGrabber_subsystem() {
    Compressor objCompressor = new Compressor(2, PneumaticsModuleType.CTREPCM);
    objCompressor.enableDigital();;
    // objSolenoid = new Solenoid(ModuleType.kRev, 2);
    objSolenoid = new Solenoid(PneumaticsModuleType.CTREPCM, Constants.Grabber.iChannel);
  }

  public void closeGrabber() {
    objSolenoid.set(true);
  }

  public void openGrabber() {
    objSolenoid.set(false);
  }

  public void toggleGrabber() {
    bState = objSolenoid.get();
    objSolenoid.set(!bState);


  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
