package frc.robot.commands.Autonomous;

//Mimicing code found using link below
//https://github.com/TitaniumTigers4829/loopy-robot-code-2022/blob/main/src/main/java/frc/robot/commands/autonomous/FollowTrajectoryPathPlanner.java

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import edu.wpi.first.math.controller.PIDController;
// import edu.wpi.first.math.controller.ProfiledPIDController;      // Unused
// import edu.wpi.first.math.trajectory.Trajectory;                 // Unused
import edu.wpi.first.wpilibj2.command.CommandBase;
// import edu.wpi.first.wpilibj2.command.Subsystem;                 // Unused
import frc.robot.Constants;                                         // Changed to Constants.java
import frc.robot.Constants.PathPlannerConstants;
import frc.robot.subsystems.Swerve_subsystem;                                 // Changed to our Swerve subsystem

public class FollowTrajectoryPathPlanner extends CommandBase {

    private Swerve_subsystem swerveSubsystem;
    private String pathName;
    private boolean zeroInitialPose;
    
    PPSwerveControllerCommand followTrajectoryPathPlannerCommand;
    private boolean done = false;
    
    /** Creates a new FollowTrajectoryPathPlanner. */
    public FollowTrajectoryPathPlanner(Swerve_subsystem swerveSubsystem_in, String pathName_in, boolean zeroInitialPose_in) {
        swerveSubsystem = swerveSubsystem_in;
        addRequirements(swerveSubsystem_in);
    
        pathName = pathName_in;
        zeroInitialPose = zeroInitialPose_in;
    }
    
    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
                                                    
        PathPlannerTrajectory trajectoryToFollow = PathPlanner.loadPath(pathName, PathPlannerConstants.autoMaxVelocity, PathPlannerConstants.autoMaxAcceleration);
    
        // Resets the pose of the robot if true (should generally only be true for the first path of an auto)
        if (zeroInitialPose) {
            swerveSubsystem.resetOdometry(trajectoryToFollow.getInitialPose());
        // Swerve.resetOdometry(trajectoryToFollow.getInitialPose());
        }

        // PID controllers
        PIDController xController = new PIDController(PathPlannerConstants.kPXController, 0, 0);
        PIDController yController = new PIDController(PathPlannerConstants.kPYController, 0, 0);
        PIDController thetaController = new PIDController(PathPlannerConstants.kPThetaController, 0, 0);
        // ProfiledPIDController thetaController = new ProfiledPIDController(
        // PathPlannerConstants.kPThetaController, 0, 0, PathPlannerConstants.kThetaControllerConstraints);
        thetaController.enableContinuousInput(-Math.PI, Math.PI); // Makes it so wheels don't have to turn more than 90 degrees

        // Create a PPSwerveControllerCommand. This is almost identical to WPILib's SwerveControllerCommand, but it uses the holonomic rotation from the PathPlannerTrajectory to control the robot's rotation.
        followTrajectoryPathPlannerCommand = new PPSwerveControllerCommand(
            trajectoryToFollow,
            swerveSubsystem::getPose, // Functional interface to feed supplier
            // DriveConstants.kDriveKinematics,
            Constants.Swerve.swerveKinematics,          //Changed by Sequence
            xController, 
            yController, 
            thetaController, 
            swerveSubsystem::setModuleStates, 
            true, 
            swerveSubsystem
        );
        // followTrajectoryPathPlannerCommand = new PPSwerveControllerCommand(trajectoryToFollow, swerveSubsystem::getPose, xController, yController, yController, swerveSubsystem::setModuleStates, swerveSubsystem);
        
        followTrajectoryPathPlannerCommand.schedule();

    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        done = followTrajectoryPathPlannerCommand.isFinished();
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {}

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return done;
    }

}
