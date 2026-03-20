package frc.robot;

import java.util.stream.IntStream;

import choreo.auto.AutoFactory;
import choreo.trajectory.SwerveSample;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest.ForwardPerspectiveValue;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.GeneralConstants;
import frc.robot.generated.ChoreoTraj;
import frc.robot.generated.ChoreoVars;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.util.Utilities;

public class Autos extends SubsystemBase
{
    private enum AutoMode
    {
        DoNothing("Do Nothing"), DriveOnly("Drive Only"), ShootOnly("Shoot Only"), ShootWithDelay("Shoot + Delay"), ShootThenDrive("Shoot Then Drive"), ShootWithDelayThenDrive("Shoot + Delay + Drive");

        public final String displayName;

        private AutoMode(String displayName)
        {
            this.displayName = displayName;
        }

        public boolean usesDrivePath()
        {
            return this == DriveOnly || this == ShootThenDrive || this == ShootWithDelayThenDrive;
        }

        public boolean usesDelay()
        {
            return this == ShootWithDelay || this == ShootWithDelayThenDrive;
        }
    }

    private record AutoDriveOption(String displayName, ChoreoTraj trajectory)
    {
        boolean staysPut()
        {
            return trajectory == null;
        }

        String key()
        {
            return staysPut() ? STAY_PUT_KEY : trajectory.name();
        }

        Pose2d endPoseBlue()
        {
            return staysPut() ? null : trajectory.endPoseBlue();
        }
    }

    private enum StartPosition
    {
        LeftTrench("Left Trench", ChoreoVars.Poses.LeftTrench, new AutoDriveOption("Stay Put", null), new AutoDriveOption("Drive To Depot", ChoreoTraj.LeftTrenchToDepot)),
        LeftBump("Left Bump", ChoreoVars.Poses.LeftBump, new AutoDriveOption("Stay Put", null), new AutoDriveOption("Drive To Depot", ChoreoTraj.LeftBumpToDepot), new AutoDriveOption("Drive To Tower", ChoreoTraj.LeftBumpToTower),
                new AutoDriveOption("Pickup From Mid", ChoreoTraj.LeftBumpPickupFromMidScore)),
        Hub("Hub", ChoreoVars.Poses.HubStart, new AutoDriveOption("Stay Put", null), new AutoDriveOption("Drive To Depot", ChoreoTraj.HubToDepot), new AutoDriveOption("Drive To Outpost", ChoreoTraj.HubToOutpost),
                new AutoDriveOption("Drive To Tower", ChoreoTraj.HubToTower)),
        RightBump("Right Bump", ChoreoVars.Poses.RightBump, new AutoDriveOption("Stay Put", null), new AutoDriveOption("Drive To Outpost", ChoreoTraj.RightBumpToOutpost), new AutoDriveOption("Drive To Tower", ChoreoTraj.RightBumpToTower),
                new AutoDriveOption("Pickup From Mid", ChoreoTraj.RightBumpPickupFromMidScore)),
        RightTrench("Right Trench", ChoreoVars.Poses.RightTrench, new AutoDriveOption("Stay Put", null), new AutoDriveOption("Drive To Outpost", ChoreoTraj.RightTrenchToOutpost));

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

    private static final String              STAY_PUT_KEY            = "__STAY_PUT__";
    private static final String              AUTO_MODE_CHOOSER_NAME  = "Auto Mode";
    private static final String              AUTO_START_CHOOSER_NAME = "Auto Start Position";
    private static final String              AUTO_DRIVE_CHOOSER_NAME = "Auto Drive Path";
    private static final String              AUTO_DELAY_CHOOSER_NAME = "Auto Delay";
    private static final AutoMode            DEFAULT_AUTO_MODE       = AutoMode.ShootOnly;
    private static final StartPosition       DEFAULT_START_POSITION  = StartPosition.Hub;
    private static final int                 DEFAULT_DELAY_SECONDS   = 0;
    private final Drive                      _driveSubsystem;
    private final Shooter                    _shooterSubsystem;
    private final AutoFactory                _autoFactory;
    private final SwerveRequest.FieldCentric _autoRequest            = new SwerveRequest.FieldCentric().withDriveRequestType(DriveRequestType.OpenLoopVoltage).withForwardPerspective(ForwardPerspectiveValue.BlueAlliance);
    private final PIDController              _xController            = new PIDController(0.0, 0.0, 0.0);
    private final PIDController              _yController            = new PIDController(0.0, 0.0, 0.0);
    private final PIDController              _headingController      = new PIDController(0.0, 0.0, 0.0);
    private final SendableChooser<String>    _modeChooser            = new SendableChooser<>();
    private final SendableChooser<String>    _startChooser           = new SendableChooser<>();
    private final SendableChooser<String>    _delayChooser           = new SendableChooser<>();
    private SendableChooser<String>          _driveChooser           = new SendableChooser<>();
    private final Field2d                    _previewField           = new Field2d();
    private StartPosition                    _lastStartPosition      = null;

    public Autos(Drive driveSubsystem, Shooter shooterSubsystem)
    {
        _driveSubsystem   = driveSubsystem;
        _shooterSubsystem = shooterSubsystem;

        _headingController.enableContinuousInput(-Math.PI, Math.PI);
        _autoFactory = new AutoFactory(() -> driveSubsystem.getState().Pose, driveSubsystem::resetPose, this::followTrajectory, true, driveSubsystem);

        configureChoosers();
        rebuildDriveChooser(DEFAULT_START_POSITION);
        updateDashboard();
    }

    @Override
    public void periodic()
    {
        var selectedStart = getSelectedStartPosition();
        if (_lastStartPosition != selectedStart)
        {
            rebuildDriveChooser(selectedStart);
        }

        updateDashboard();
    }

    public Command buildAuto()
    {
        var mode          = getSelectedMode();
        var startPosition = getSelectedStartPosition();
        var delaySeconds  = getSelectedDelaySeconds();
        var driveOption   = getSelectedDriveOption(startPosition);
        var startPose     = flip(startPosition.blueStartPose);
        var resetPose     = Commands.runOnce(() -> _driveSubsystem.resetPose(startPose));
        var shoot         = _shooterSubsystem.shoot().withTimeout(4.0);
        var delay         = Commands.waitSeconds(delaySeconds);
        var drive         = driveOption.staysPut() ? Commands.none() : _autoFactory.trajectoryCmd(driveOption.trajectory().name());

        return switch (mode)
        {
            case DoNothing -> resetPose;
            case DriveOnly -> Commands.sequence(resetPose, drive);
            case ShootOnly -> Commands.sequence(resetPose, shoot);
            case ShootWithDelay -> Commands.sequence(resetPose, delay, shoot);
            case ShootThenDrive -> Commands.sequence(resetPose, shoot, drive);
            case ShootWithDelayThenDrive -> Commands.sequence(resetPose, delay, shoot, drive);
        };
    }

    private void configureChoosers()
    {
        _modeChooser.setDefaultOption(DEFAULT_AUTO_MODE.displayName, DEFAULT_AUTO_MODE.name());
        for (var mode : AutoMode.values())
        {
            if (mode != DEFAULT_AUTO_MODE)
            {
                _modeChooser.addOption(mode.displayName, mode.name());
            }
        }

        _startChooser.setDefaultOption(DEFAULT_START_POSITION.displayName, DEFAULT_START_POSITION.name());
        for (var startPosition : StartPosition.values())
        {
            if (startPosition != DEFAULT_START_POSITION)
            {
                _startChooser.addOption(startPosition.displayName, startPosition.name());
            }
        }

        _delayChooser.setDefaultOption(String.valueOf(DEFAULT_DELAY_SECONDS), String.valueOf(DEFAULT_DELAY_SECONDS));
        IntStream.rangeClosed(1, 5).forEach(seconds -> _delayChooser.addOption(String.valueOf(seconds), String.valueOf(seconds)));

        SmartDashboard.putData(AUTO_MODE_CHOOSER_NAME, _modeChooser);
        SmartDashboard.putData(AUTO_START_CHOOSER_NAME, _startChooser);
        SmartDashboard.putData(AUTO_DELAY_CHOOSER_NAME, _delayChooser);
        SmartDashboard.putData(AUTO_DRIVE_CHOOSER_NAME, _driveChooser);
        SmartDashboard.putData("Autonomous Preview", _previewField);
    }

    private void rebuildDriveChooser(StartPosition startPosition)
    {
        var chooser    = new SendableChooser<String>();
        var options    = startPosition.driveOptions;
        var defaultKey = firstDrivePathKey(startPosition);

        for (var option : options)
        {
            if (option.key().equals(defaultKey))
            {
                chooser.setDefaultOption(option.displayName(), option.key());
            }
            else
            {
                chooser.addOption(option.displayName(), option.key());
            }
        }

        _driveChooser      = chooser;
        _lastStartPosition = startPosition;
        SmartDashboard.putData(AUTO_DRIVE_CHOOSER_NAME, _driveChooser);
    }

    private void updateDashboard()
    {
        var mode          = getSelectedMode();
        var startPosition = getSelectedStartPosition();
        var driveOption   = getSelectedDriveOption(startPosition);
        var startPose     = flip(startPosition.blueStartPose);
        var endPose       = driveOption.staysPut() ? startPose : flip(driveOption.endPoseBlue());

        _previewField.setRobotPose(startPose);
        _previewField.getObject("Auto End Pose").setPose(endPose);

        SmartDashboard.putString("Auto Summary", buildSummary(mode, startPosition, driveOption, getSelectedDelaySeconds()));
    }

    private AutoMode getSelectedMode()
    {
        var selected = _modeChooser.getSelected();
        if (selected == null || selected.isEmpty())
        {
            return DEFAULT_AUTO_MODE;
        }

        for (var mode : AutoMode.values())
        {
            if (mode.name().equals(selected))
            {
                return mode;
            }
        }

        return DEFAULT_AUTO_MODE;
    }

    private StartPosition getSelectedStartPosition()
    {
        var selected = _startChooser.getSelected();
        if (selected == null || selected.isEmpty())
        {
            return DEFAULT_START_POSITION;
        }

        for (var startPosition : StartPosition.values())
        {
            if (startPosition.name().equals(selected))
            {
                return startPosition;
            }
        }

        return DEFAULT_START_POSITION;
    }

    private int getSelectedDelaySeconds()
    {
        var selected = _delayChooser.getSelected();
        if (selected == null || selected.isEmpty())
        {
            return DEFAULT_DELAY_SECONDS;
        }

        try
        {
            return Integer.parseInt(selected);
        }
        catch (NumberFormatException exception)
        {
            return DEFAULT_DELAY_SECONDS;
        }
    }

    private AutoDriveOption getSelectedDriveOption(StartPosition startPosition)
    {
        var fallback = firstDrivePathKey(startPosition);
        var selected = _driveChooser.getSelected();
        if (selected == null || selected.isEmpty())
        {
            selected = fallback;
        }

        for (var option : startPosition.driveOptions)
        {
            if (option.key().equals(selected))
            {
                return option;
            }
        }

        for (var option : startPosition.driveOptions)
        {
            if (option.key().equals(fallback))
            {
                return option;
            }
        }

        return startPosition.driveOptions[0];
    }

    private String buildSummary(AutoMode mode, StartPosition startPosition, AutoDriveOption driveOption, int delaySeconds)
    {
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

    private String firstDrivePathKey(StartPosition startPosition)
    {
        for (var option : startPosition.driveOptions)
        {
            if (!option.staysPut())
            {
                return option.key();
            }
        }

        return startPosition.driveOptions[0].key();
    }

    private void followTrajectory(SwerveSample sample)
    {
        var pose = _driveSubsystem.getState().Pose;

        _driveSubsystem.setControl(
                _autoRequest.withVelocityX(sample.vx + _xController.calculate(pose.getX(), sample.x)).withVelocityY(sample.vy + _yController.calculate(pose.getY(), sample.y))
                        .withRotationalRate(sample.omega + _headingController.calculate(pose.getRotation().getRadians(), sample.heading))
        );
    }

    private Pose2d flip(Pose2d bluePose)
    {
        if (Utilities.isRedAlliance())
        {
            return bluePose.rotateAround(new Translation2d(GeneralConstants.FIELD_SIZE_X.div(2), GeneralConstants.FIELD_SIZE_Y.div(2)), Rotation2d.k180deg);
        }

        return bluePose;
    }
}
