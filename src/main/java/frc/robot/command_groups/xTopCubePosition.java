// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.command_groups;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
//Stuff we imported
import frc.robot.subsystems.*;

import frc.robot.Constants;
import frc.robot.commands.Arm_command;
import frc.robot.commands.Forearm_command;
import frc.robot.commands.Wrist_command;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class xTopCubePosition extends SequentialCommandGroup {

  // Target Positions
  double dArmTarget = 9.9;
  double dForearmTarget = 62.1;
  double dWristTarget = 8.5;
  //Raw Wrist = 1.4

  /** Creates a new Mech_cmd_group. */
  public xTopCubePosition(Arm_subsystem obj_Arm, Forearm_subsystem obj_Forearm, Wrist_subsystem obj_Wrist) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new ParallelCommandGroup(
        new Arm_command(obj_Arm, Constants.Arm.dArmSpeedManual, true, dArmTarget).withTimeout(5.0),
        new Forearm_command(obj_Forearm, Constants.Forearm.dSpeedManual, true, dForearmTarget),
        new SequentialCommandGroup(
          new WaitCommand(4),
          new Wrist_command(obj_Wrist, Constants.Wrist.dSpeedManual, true, dWristTarget)
        )
      ),
      new Forearm_command(obj_Forearm, Constants.Forearm.dSpeedManual, true, dForearmTarget)

    );
  }
}
