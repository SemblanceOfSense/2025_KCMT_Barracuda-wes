package frc.robot.commands.swervedrive.drivebase;

import java.util.Optional;

import org.photonvision.EstimatedRobotPose;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import frc.robot.subsystems.swervedrive.Vision;
import frc.robot.subsystems.swervedrive.Vision.Cameras;

public class DriveToPoint extends Command {
    private final SwerveSubsystem swerveSubsystem;
    private final Vision odometry;
    private final Cameras cam;
    private final Pose2d desiredPose;
    public boolean isAtPoint;
    // TEMPORARY
    private final double MAX_VEL;
    private final double MAX_ANG_VEL;

    public DriveToPoint(SwerveSubsystem swerveSubsystem, Vision odometry, Cameras cam, Pose2d desiredPose) {
        this.swerveSubsystem = swerveSubsystem;
        this.odometry = odometry;
        this.cam = cam;
        this.desiredPose = desiredPose;
    }

    @Override
    public void execute() {
        EstimatedRobotPose currentPose;
        Optional<EstimatedRobotPose> currentPoseCheck = this.odometry.getEstimatedGlobalPose(cam);
        if (!currentPoseCheck.isPresent()) {
            System.out.println("Could not estimate global pose");
            isAtPoint = false;
            return;
        } else {
            currentPose = currentPoseCheck.get();
        }

        // TODO: Change 0, 0, 0 to diff values
        PIDController drivePid = new PIDController(0, 0, 0);
        Translation2d translation = new Translation2d(
                MAX_VEL * drivePid.calculate(desiredPose.getX() - currentPose.estimatedPose.toPose2d().getX()),
                MAX_VEL * drivePid.calculate(desiredPose.getY() - currentPose.estimatedPose.toPose2d().getX()));
        double angVel = MAX_ANG_VEL * drivePid.calculate(desiredPose.getRotation().getRotations() - currentPose.estimatedPose.toPose2d().getRotation().getRotations());

        swerveSubsystem.drive(translation, angVel, true);
    }
}
