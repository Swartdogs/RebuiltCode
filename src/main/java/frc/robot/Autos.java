package frc.robot;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest.ForwardPerspectiveValue;

import java.util.stream.IntStream;

import com.ctre.phoenix6.swerve.SwerveRequest;

import choreo.auto.AutoFactory;
import choreo.trajectory.SwerveSample;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.Drive;

public class Autos
{
    private enum StartPosition
    {
        LeftTrench("Left Trench"), LeftBump("Left Bump"), Hub("Hub"), RightBump("Right Bump"), RightTrench("Right Trench");

        private final String displayName;

        private StartPosition(String displayName)
        {
            this.displayName = displayName;
        }
    }

    private enum AllianceFuelSourceSelection
    {
        None("None"), Depot("Depot"), Outpost("Outpost"), DepotToOutpost("Depot to Outpost"), OutpostToDepot("Outpost to Depot");

        private final String displayName;

        private AllianceFuelSourceSelection(String displayName)
        {
            this.displayName = displayName;
        }
    }

    private final AutoFactory                _autoFactory;
    private final Drive                      _driveSubsystem;
    private final SwerveRequest.FieldCentric _autoFollowingRequest = new SwerveRequest.FieldCentric().withDriveRequestType(DriveRequestType.OpenLoopVoltage);
    private final PIDController              _xController          = new PIDController(0.0, 0.0, 0.0);
    private final PIDController              _yController          = new PIDController(0.0, 0.0, 0.0);
    private final PIDController              _headingController    = new PIDController(0.0, 0.0, 0.0);

    // Dashboard
    private final Field2d                                      _field;
    private final SendableChooser<Integer>                     _autoDelayChooser;
    private final SendableChooser<StartPosition>               _startPositionChooser;
    private final SendableChooser<AllianceFuelSourceSelection> _allianceFuelSourceSelectionChooser;
    private final String                                       _neutralZoneNTKey;
    private final String                                       _climbNTKey;

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

        // Dashboard
        _field = new Field2d();

        _autoDelayChooser                   = new SendableChooser<>();
        _startPositionChooser               = new SendableChooser<>();
        _allianceFuelSourceSelectionChooser = new SendableChooser<>();

        _neutralZoneNTKey = "Neutral Zone";
        _climbNTKey       = "Climb";

        _autoDelayChooser.setDefaultOption("0", 0);
        IntStream.range(1, 6).forEach(n -> _autoDelayChooser.addOption(String.valueOf(n), n));

        var startPositions = StartPosition.values();
        var fuelSources    = AllianceFuelSourceSelection.values();

        _startPositionChooser.setDefaultOption(startPositions[0].displayName, startPositions[0]);
        _allianceFuelSourceSelectionChooser.setDefaultOption(fuelSources[0].displayName, fuelSources[0]);

        for (int i = 1; i < startPositions.length; i++)
        {
            _startPositionChooser.addOption(startPositions[i].displayName, startPositions[i]);
        }

        for (int i = 1; i < fuelSources.length; i++)
        {
            _allianceFuelSourceSelectionChooser.addOption(fuelSources[i].displayName, fuelSources[i]);
        }

        SmartDashboard.putData("Autonomous Mode", _field);
        SmartDashboard.putData("Auto Delay", _autoDelayChooser);
        SmartDashboard.putData("Start Position", _startPositionChooser);
        SmartDashboard.putData("Alliance Fuel Source", _allianceFuelSourceSelectionChooser);
        SmartDashboard.putData("Auto X PID", _xController);
        SmartDashboard.putData("Auto Y PID", _yController);
        SmartDashboard.putData("Auto Heading PID", _headingController);
        SmartDashboard.putBoolean(_neutralZoneNTKey, false);
        SmartDashboard.putBoolean(_climbNTKey, false);
    }

    private void followTrajectory(SwerveSample sample)
    {
        // Get the current pose of the robot
        Pose2d pose = _driveSubsystem.getState().Pose;

        // Build up "request" based on "sample"
        // @formatter:off
        _autoFollowingRequest
            .withVelocityX(sample.vx + _xController.calculate(pose.getX(), sample.x))
            .withVelocityY(sample.vy + _yController.calculate(pose.getY(), sample.y))
            .withRotationalRate(sample.omega + _headingController.calculate(pose.getRotation().getRadians(), sample.heading))
            .withForwardPerspective(ForwardPerspectiveValue.BlueAlliance);
        // @formatter:on

        _driveSubsystem.setControl(_autoFollowingRequest);
    }

    public Command followPath(String pathName)
    {
        // @formatter:off
        return Commands.sequence
        (
            _autoFactory.resetOdometry(pathName),
            _autoFactory.trajectoryCmd(pathName),
            Commands.runOnce(() -> _driveSubsystem.setControl(new SwerveRequest.Idle()), _driveSubsystem)
        );
        // @formatter:on
    }
}
