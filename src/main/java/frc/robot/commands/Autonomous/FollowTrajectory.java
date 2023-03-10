package frc.robot.commands.Autonomous;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants;                                         //Changed to our constant class
import frc.robot.subsystems.Swerve_subsystem;                                 //Changed to our swerve subsystem
import java.io.IOException;
import java.nio.file.Path;

public class FollowTrajectory extends CommandBase {

  private final Swerve_subsystem drive;
  private Trajectory trajectory;
  private boolean toReset;

  public FollowTrajectory(Swerve_subsystem drive, String trajectoryFilePath, boolean toReset) {
    this.drive = drive;
    this.toReset = toReset;
    addRequirements(drive);

    try {
      Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(trajectoryFilePath);
      trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
    } catch (IOException e) {
      DriverStation.reportError("Unable to open trajectory: " + trajectoryFilePath,
          e.getStackTrace());
    }
  }

  @Override
  public void initialize() {
    if (toReset) {
      drive.resetOdometry(trajectory.getInitialPose());
    }

    final ProfiledPIDController thetaController =
        new ProfiledPIDController(
            AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    new SwerveControllerCommand(       //TODO: Fix Errors
        trajectory,
        drive::getPose, // Functional interface to feed supplier
        Constants.Swerve.swerveKinematics,

        // Position controllers
        new PIDController(AutoConstants.kPXController, 0, 0),
        new PIDController(AutoConstants.kPYController, 0, 0),
        thetaController,
        drive::setModuleStates,
        drive).andThen(() -> drive.drive(Constants.Swerve.zeroTranslation2d, 0.0, true, false)).schedule(); // Stops the robot 

        // Reset odometry to the starting pose of the trajectory.
        // drive.resetOdometry(trajectory.getInitialPose());
  }

  @Override
  public void execute() {}

  @Override      //TODO: Fix Errors
  public void end(boolean interrupted) {
    drive.drive(Constants.Swerve.zeroTranslation2d, 0.0, true, false);
  }

  @Override
  public boolean isFinished() {
    return false;
  }

}

