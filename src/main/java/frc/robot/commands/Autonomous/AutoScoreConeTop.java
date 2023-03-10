// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autonomous;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

// our Imports
import frc.robot.commands.Button_commands.xScoreConeTop2;
import frc.robot.subsystems.Arm_subsystem;
import frc.robot.subsystems.Forearm_subsystem;
import frc.robot.subsystems.Wrist_subsystem;
import frc.robot.commands.xGrabber_command;

import frc.robot.commands.Arm_command;
import frc.robot.commands.Forearm_command;
import frc.robot.commands.Wrist_command;

import frc.robot.subsystems.xGrabber_subsystem;



// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoScoreConeTop extends SequentialCommandGroup {
  /** Creates a new AutoScoreConeTop. */
  public AutoScoreConeTop(Arm_subsystem objArm, Forearm_subsystem objForearm, Wrist_subsystem objWrist, xGrabber_subsystem objGrabber) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new xScoreConeTop2(objArm, objForearm, objWrist).withTimeout(3.0),
      new xGrabber_command(objGrabber)
    );


  }
}
