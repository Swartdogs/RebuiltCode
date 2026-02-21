package frc.robot;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import choreo.auto.AutoFactory;
import choreo.trajectory.SwerveSample;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.Drive;

public class Autos
{
    private final AutoFactory                _autoFactory;
    private final Drive                      _driveSubsystem;
    private final SwerveRequest.FieldCentric _autoFollowingRequest = new SwerveRequest.FieldCentric().withDriveRequestType(DriveRequestType.OpenLoopVoltage);
    private final PIDController              _xController          = new PIDController(0.0, 0.0, 0.0);
    private final PIDController              _yController          = new PIDController(0.0, 0.0, 0.0);
    private final PIDController              _headingController    = new PIDController(0.0, 0.0, 0.0);

    public Autos(Drive driveSubsystem)
    {
        _driveSubsystem = driveSubsystem;

        // @formatter:off
        _autoFactory = new AutoFactory
        (
            () -> driveSubsystem.getState().Pose,
            driveSubsystem::resetPose,
            this::followTrajectory,
            true,
            driveSubsystem
        );
        // @formatter:on
    }

    private void followTrajectory(SwerveSample sample)
    {
        // Get the current pose of the robot
        Pose2d pose = _driveSubsystem.getState().Pose;

        // Build up "request" based on "sample"
        _autoFollowingRequest.withVelocityX(sample.vx + _xController.calculate(pose.getX(), sample.x)).withVelocityY(sample.vy + _yController.calculate(pose.getY(), sample.y))
                .withRotationalRate(sample.omega + _headingController.calculate(pose.getRotation().getRadians(), sample.heading));

        _driveSubsystem.setControl(_autoFollowingRequest);
    }

    public Command followPath(String pathName)
    {
        return Commands.sequence(_autoFactory.resetOdometry(pathName), _autoFactory.trajectoryCmd(pathName), Commands.runOnce(() -> _driveSubsystem.setControl(new SwerveRequest.Idle()), _driveSubsystem));
    }
}
