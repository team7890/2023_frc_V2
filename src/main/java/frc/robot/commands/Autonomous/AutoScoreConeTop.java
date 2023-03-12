// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autonomous;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.General_Movement_Commands.Arm_command;
import frc.robot.commands.General_Movement_Commands.Forearm_command;
import frc.robot.commands.General_Movement_Commands.Wrist_command;
import frc.robot.subsystems.RollerHand_subsystem;
// our Imports
import frc.robot.subsystems.Arm_subsystem;
import frc.robot.subsystems.Forearm_subsystem;
import frc.robot.subsystems.Wrist_subsystem;
import frc.robot.commands.Roller_commands.ConeOuttake_command;


// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoScoreConeTop extends SequentialCommandGroup {
  /** Creates a new AutoScoreConeTop. */
  public AutoScoreConeTop(Arm_subsystem objArm, Forearm_subsystem objForearm, Wrist_subsystem objWrist, RollerHand_subsystem objRollerHand) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      // new xScoreConeTop2(objArm, objForearm, objWrist).withTimeout(3.0),
      new ConeOuttake_command(objRollerHand)
    );


  }
}
