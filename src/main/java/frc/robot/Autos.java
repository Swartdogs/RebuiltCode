package frc.robot;

import java.util.Arrays;
import java.util.Collections;
import java.util.List;
import java.util.ArrayList;
import java.util.stream.IntStream;

import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.trajectory.SwerveSample;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest.ForwardPerspectiveValue;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.NotLogged;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.GeneralConstants;
import frc.robot.generated.ChoreoTraj;
import frc.robot.generated.ChoreoVars;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.util.Utilities;

@Logged
public class Autos extends SubsystemBase
{
    private enum AutoMode
    {
        // @formatter:off
        DoNothing("Do Nothing"),
        DriveOnly("Drive Only"),
        ShootOnly("Shoot Only"),
        ShootWithDelay("Shoot + Delay"),
        DriveThenShoot("Drive Then Shoot"),
        ShootWithDelayThenDrive("Shoot + Delay + Drive"),
        HubOutpostDelayTowerShoot("Hub -> Outpost -> 4s -> Tower -> Shoot"),
        DrivePickupThenShoot("Drive + Pickup + Shoot"),
        DriveWithDelayPickupThenShoot("Drive + Delay + Pickup + Shoot");
        // @formatter:on

        public final String displayName;

        private AutoMode(String displayName)
        {
            this.displayName = displayName;
        }

        public boolean usesDrivePath()
        {
            return this == DriveOnly || this == DriveThenShoot || this == ShootWithDelayThenDrive;
        }

        public boolean usesDelay()
        {
            return this == ShootWithDelay || this == ShootWithDelayThenDrive;
        }
    }

    private record AutoDriveOption(String displayName, ChoreoTraj trajectory)
    {
        public static final AutoDriveOption STAY_PUT = new AutoDriveOption("Stay Put", null);

        public boolean staysPut()
        {
            return trajectory == null;
        }

        public Pose2d startPoseBlue()
        {
            return staysPut() ? null : trajectory.initialPoseBlue();
        }

        public Pose2d endPoseBlue()
        {
            return staysPut() ? null : trajectory.endPoseBlue();
        }
    }

    private enum StartPosition
    {
        // @formatter:off
        LeftTrench(
            "Left Trench",
            ChoreoVars.Poses.LeftTrench,
            new AutoDriveOption("Drive To Depot", ChoreoTraj.LeftTrenchToDepot)
        ),

        LeftBump(
            "Left Bump",
            ChoreoVars.Poses.LeftBump,
            new AutoDriveOption("Drive To Depot", ChoreoTraj.LeftBumpToDepot),
            new AutoDriveOption("Drive To Tower", ChoreoTraj.LeftBumpToTower),
            new AutoDriveOption("Pickup From Mid", ChoreoTraj.LeftBumpPickupFromMidScore)
        ),

        Hub(
            "Hub",
            ChoreoVars.Poses.HubStart,
            new AutoDriveOption("Drive To Depot", ChoreoTraj.HubToDepot),
            new AutoDriveOption("Drive To Outpost", ChoreoTraj.HubToOutpost),
            new AutoDriveOption("Drive To Tower", ChoreoTraj.HubToTower)
        ),

        RightBump(
            "Right Bump",
            ChoreoVars.Poses.RightBump,
            new AutoDriveOption("Drive To Outpost", ChoreoTraj.RightBumpToOutpost),
            new AutoDriveOption("Drive To Tower", ChoreoTraj.RightBumpToTower),
            new AutoDriveOption("Pickup From Mid", ChoreoTraj.RightBumpPickupFromMidScore)
        ),

        RightTrench(
            "Right Trench",
            ChoreoVars.Poses.RightTrench,
            new AutoDriveOption("Drive To Outpost", ChoreoTraj.RightTrenchToOutpost)
        );
        // @formatter:on

        public final String            displayName;
        public final Pose2d            blueStartPose;
        public final AutoDriveOption[] driveOptions;

        private StartPosition(String displayName, Pose2d blueStartPose, AutoDriveOption... driveOptions)
        {
            this.displayName   = displayName;
            this.blueStartPose = blueStartPose;
            this.driveOptions  = driveOptions;
        }
    }

    @NotLogged
    private static final AutoMode                DEFAULT_AUTO_MODE       = AutoMode.ShootOnly;
    @NotLogged
    private static final StartPosition           DEFAULT_START_POSITION  = StartPosition.Hub;
    @NotLogged
    private static final int                     DEFAULT_DELAY_SECONDS   = 0;
    @NotLogged
    private static final int                     HUB_OUTPOST_TOWER_DELAY = 4;
    @NotLogged
    private final Drive                          _driveSubsystem;
    @NotLogged
    private final Shooter                        _shooterSubsystem;
    @NotLogged
    private final Intake                         _intakeSubsystem;
    @NotLogged
    private final AutoFactory                    _autoFactory;
    @NotLogged
    private final AutoRoutine                    _trajectoryDrawRoutine;
    @NotLogged
    private final SwerveRequest.FieldCentric     _autoRequest            = new SwerveRequest.FieldCentric().withDriveRequestType(DriveRequestType.OpenLoopVoltage).withForwardPerspective(ForwardPerspectiveValue.BlueAlliance);
    @NotLogged
    private final PIDController                  _xController            = new PIDController(AutoConstants.DRIVE_KP, 0.0, AutoConstants.DRIVE_KD);
    @NotLogged
    private final PIDController                  _yController            = new PIDController(AutoConstants.DRIVE_KP, 0.0, AutoConstants.DRIVE_KD);
    @NotLogged
    private final PIDController                  _headingController      = new PIDController(AutoConstants.ROTATE_KP, 0.0, AutoConstants.ROTATE_KD);
    @NotLogged
    private final SendableChooser<AutoMode>      _modeChooser            = new SendableChooser<>();
    @NotLogged
    private final SendableChooser<StartPosition> _startChooser           = new SendableChooser<>();
    @NotLogged
    private final SendableChooser<Integer>       _delayChooser           = new SendableChooser<>();
    @NotLogged
    private final Field2d                        _previewField           = new Field2d();
    @NotLogged
    private SendableChooser<AutoDriveOption>     _driveChooser           = new SendableChooser<>();
    @NotLogged
    private StartPosition                        _lastStartPosition      = null;
    @Logged
    private Pose2d                               _currentDesiredPathPose = new Pose2d();

    public Autos(Drive driveSubsystem, Shooter shooterSubsystem, Intake intakeSubsystem)
    {
        _driveSubsystem        = driveSubsystem;
        _shooterSubsystem      = shooterSubsystem;
        _intakeSubsystem       = intakeSubsystem;
        _autoFactory           = new AutoFactory(() -> driveSubsystem.getState().Pose, driveSubsystem::resetPose, this::followTrajectory, true, driveSubsystem);
        _trajectoryDrawRoutine = _autoFactory.newRoutine("Trajectory Drawing");

        _headingController.enableContinuousInput(-Math.PI, Math.PI);

        configureChoosers();
        rebuildDriveChooser(DEFAULT_START_POSITION);
        updateDashboard();
    }

    @Override
    public void periodic()
    {
        // Don't update the autonomous selections/display if the robot is enabled
        if (DriverStation.isEnabled())
        {
            return;
        }

        var selectedStart = _startChooser.getSelected();
        if (_lastStartPosition != selectedStart)
        {
            rebuildDriveChooser(selectedStart);
        }

        updateDashboard();
    }

    public Command buildAuto()
    {
        var mode           = _modeChooser.getSelected();
        var startPosition  = _startChooser.getSelected();
        var delaySeconds   = _delayChooser.getSelected();
        var driveOption    = _driveChooser.getSelected();
        var startPose      = getStartPose(mode, startPosition, driveOption);
        var resetPose      = Commands.runOnce(() -> _driveSubsystem.resetPose(startPose));
        var shoot          = _shooterSubsystem.shoot();
        var delay          = Commands.waitSeconds(delaySeconds);
        var extend         = _intakeSubsystem.getExtendCmd();
        var pickup         = _intakeSubsystem.runRollersForward();
        var jiggle         = _intakeSubsystem.jiggle();
        var drive          = buildDriveCommand(driveOption.trajectory());
        var hubToOutpost   = buildDriveCommand(ChoreoTraj.HubToOutpost);
        var outpostToTower = buildDriveCommand(ChoreoTraj.OutpostToTower);

        return switch (mode)
        {
            case DoNothing -> resetPose;
            case DriveOnly -> Commands.sequence(resetPose, drive);
            case ShootOnly -> Commands.sequence(resetPose, shoot);
            case ShootWithDelay -> Commands.sequence(resetPose, delay, shoot);
            case DriveThenShoot -> Commands.sequence(resetPose, drive, shoot);
            case ShootWithDelayThenDrive -> Commands.sequence(resetPose, delay, shoot, drive);
            case HubOutpostDelayTowerShoot -> Commands.sequence(resetPose, hubToOutpost, Commands.waitSeconds(HUB_OUTPOST_TOWER_DELAY), outpostToTower, shoot);
            case DrivePickupThenShoot -> Commands.sequence(resetPose, extend, Commands.deadline(drive, pickup), shoot.alongWith(jiggle));
            case DriveWithDelayPickupThenShoot -> Commands.sequence(resetPose, delay, extend, Commands.deadline(drive, pickup), shoot.alongWith(jiggle));
        };
    }

    private void configureChoosers()
    {
        _modeChooser.setDefaultOption(DEFAULT_AUTO_MODE.displayName, DEFAULT_AUTO_MODE);
        for (var mode : AutoMode.values())
        {
            if (mode != DEFAULT_AUTO_MODE)
            {
                _modeChooser.addOption(mode.displayName, mode);
            }
        }

        _startChooser.setDefaultOption(DEFAULT_START_POSITION.displayName, DEFAULT_START_POSITION);
        for (var startPosition : StartPosition.values())
        {
            if (startPosition != DEFAULT_START_POSITION)
            {
                _startChooser.addOption(startPosition.displayName, startPosition);
            }
        }

        _delayChooser.setDefaultOption(String.valueOf(DEFAULT_DELAY_SECONDS), DEFAULT_DELAY_SECONDS);
        IntStream.rangeClosed(1, 5).forEach(seconds -> _delayChooser.addOption(String.valueOf(seconds), seconds));

        SmartDashboard.putData(AutoConstants.AUTO_MODE_CHOOSER_NAME, _modeChooser);
        SmartDashboard.putData(AutoConstants.AUTO_START_CHOOSER_NAME, _startChooser);
        SmartDashboard.putData(AutoConstants.AUTO_DELAY_CHOOSER_NAME, _delayChooser);
        SmartDashboard.putData(AutoConstants.AUTO_DRIVE_CHOOSER_NAME, _driveChooser);
        SmartDashboard.putData("Autonomous Preview", _previewField);
    }

    private void rebuildDriveChooser(StartPosition startPosition)
    {
        var chooser = new SendableChooser<AutoDriveOption>();

        chooser.setDefaultOption(AutoDriveOption.STAY_PUT.displayName, AutoDriveOption.STAY_PUT);
        for (var option : startPosition.driveOptions)
        {
            chooser.addOption(option.displayName, option);
        }

        _driveChooser      = chooser;
        _lastStartPosition = startPosition;
        SmartDashboard.putData(AutoConstants.AUTO_DRIVE_CHOOSER_NAME, _driveChooser);
    }

    private void updateDashboard()
    {
        var mode          = _modeChooser.getSelected();
        var startPosition = _startChooser.getSelected();
        var driveOption   = _driveChooser.getSelected();
        var startPose     = getStartPose(mode, startPosition, driveOption);
        var endPose       = getEndPose(mode, startPose, driveOption);
        var trajectory    = getTrajectoryPreview(mode, driveOption);

        if (startPose != null)
        {
            _previewField.setRobotPose(startPose);
            _previewField.getObject("Auto End Pose").setPose(endPose);
            _previewField.getObject("Auto Trajectory").setPoses(trajectory);
        }

        SmartDashboard.putString("Auto Summary", buildSummary(mode, startPosition, driveOption, _delayChooser.getSelected()));
    }

    private String buildSummary(AutoMode mode, StartPosition startPosition, AutoDriveOption driveOption, int delaySeconds)
    {
        if (mode == AutoMode.HubOutpostDelayTowerShoot)
        {
            return "Hub -> Outpost | 4s Delay | Tower -> Shoot";
        }

        var summary = new StringBuilder();
        summary.append(mode.displayName).append(" | ").append(startPosition.displayName);

        if (mode.usesDelay())
        {
            summary.append(" | ").append(delaySeconds).append("s Delay");
        }

        if (mode.usesDrivePath())
        {
            summary.append(" | ").append(driveOption.displayName());
        }

        return summary.toString();
    }

    private void followTrajectory(SwerveSample sample)
    {
        var pose = _driveSubsystem.getState().Pose;

        _currentDesiredPathPose = sample.getPose();

        // @formatter:off
        _driveSubsystem.setControl(
            _autoRequest
                .withVelocityX(sample.vx + _xController.calculate(pose.getX(), sample.x))
                .withVelocityY(sample.vy + _yController.calculate(pose.getY(), sample.y))
                .withRotationalRate(sample.omega + _headingController.calculate(pose.getRotation().getRadians(), sample.heading))
        );
        // @formatter:on
    }

    private Pose2d getStartPose(AutoMode mode, StartPosition startPosition, AutoDriveOption driveOption)
    {
        if (mode == AutoMode.HubOutpostDelayTowerShoot)
        {
            return flip(ChoreoTraj.HubToOutpost.initialPoseBlue());
        }

        var pose = startPosition.blueStartPose;

        if (driveOption != null && (!driveOption.staysPut() || mode.usesDrivePath()))
        {
            pose = driveOption.startPoseBlue();
        }

        return flip(pose);
    }

    private Pose2d getEndPose(AutoMode mode, Pose2d startPose, AutoDriveOption driveOption)
    {
        if (mode == AutoMode.HubOutpostDelayTowerShoot)
        {
            return flip(ChoreoTraj.OutpostToTower.endPoseBlue());
        }

        return driveOption.staysPut() ? startPose : flip(driveOption.endPoseBlue());
    }

    private List<Pose2d> getTrajectoryPreview(AutoMode mode, AutoDriveOption driveOption)
    {
        if (mode == AutoMode.HubOutpostDelayTowerShoot)
        {
            var trajectory = new ArrayList<Pose2d>();
            trajectory.addAll(Arrays.asList(ChoreoTraj.HubToOutpost.asAutoTraj(_trajectoryDrawRoutine).getRawTrajectory().getPoses()));
            trajectory.addAll(Arrays.asList(ChoreoTraj.OutpostToTower.asAutoTraj(_trajectoryDrawRoutine).getRawTrajectory().getPoses()));
            return flip(trajectory);
        }

        if (!driveOption.staysPut() && mode.usesDrivePath())
        {
            return flip(Arrays.asList(driveOption.trajectory.asAutoTraj(_trajectoryDrawRoutine).getRawTrajectory().getPoses()));
        }

        return Collections.emptyList();
    }

    private Command buildDriveCommand(ChoreoTraj trajectory)
    {
        if (trajectory == null)
        {
            return Commands.none();
        }

        return _autoFactory.trajectoryCmd(trajectory.name()).andThen(_driveSubsystem.runOnce(() -> _driveSubsystem.setControl(_autoRequest.withVelocityX(0).withVelocityY(0).withRotationalRate(0))));
    }

    private List<Pose2d> flip(List<Pose2d> bluePoses)
    {
        if (Utilities.isRedAlliance())
        {
            return bluePoses.stream().map(p -> p.rotateAround(GeneralConstants.FIELD_CENTER, Rotation2d.k180deg)).toList();
        }

        return bluePoses;
    }

    private Pose2d flip(Pose2d bluePose)
    {
        if (bluePose == null)
        {
            return null;
        }

        if (Utilities.isRedAlliance())
        {
            return bluePose.rotateAround(GeneralConstants.FIELD_CENTER, Rotation2d.k180deg);
        }

        return bluePose;
    }
}
