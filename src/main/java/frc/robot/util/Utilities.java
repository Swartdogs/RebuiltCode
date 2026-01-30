package frc.robot.util;

import java.util.List;
import java.util.stream.Stream;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

import frc.robot.Constants.ShooterConstants;

/**
 * Utility class for reading REBUILT 2026 game data from the Driver Station /
 * FMS.
 * <p>
 * In 2026, which HUB is active during each Alliance Shift is determined by the
 * alliance that scores more FUEL in Auto. That result is sent to robots via
 * DriverStation game data.
 */
public final class Utilities
{
    private static final double       kTransitionShiftEndTimeSecs = 130.0; // 2:10
    private static final double       kShift1EndTimeSecs          = 105.0; // 1:45
    private static final double       kShift2EndTimeSecs          = 80.0;  // 1:20
    private static final double       kShift3EndTimeSecs          = 55.0;  // 0:55
    private static final double       kShift4EndTimeSecs          = 30.0;  // 0:30
    private static final List<Double> kAllHubTagIds               = Stream.concat(ShooterConstants.RED_HUB_TAG_IDS.stream(), ShooterConstants.BLUE_HUB_TAG_IDS.stream()).toList();

    private Utilities()
    {
    }

    public static boolean isHubActive()
    {
        double timeRemaining = DriverStation.getMatchTime();
        if (timeRemaining < 0.0)
        {
            return true;
        }

        if (DriverStation.isAutonomous()) return true;
        if (!DriverStation.isTeleop()) return true;

        int shift = getAllianceShift(timeRemaining);
        if (shift == 0 || shift == 5) return true;

        String   gameData      = DriverStation.getGameSpecificMessage();
        Alliance inactiveFirst = getInactiveFirstAlliance(gameData);
        if (inactiveFirst == null) return true;

        var alliance = DriverStation.getAlliance();
        if (alliance.isEmpty()) return true;

        boolean inactiveFirstActiveShift = (shift % 2) == 0; // Shifts 2 and 4
        if (alliance.get() == inactiveFirst)
        {
            return inactiveFirstActiveShift;
        }
        else
        {
            return !inactiveFirstActiveShift;
        }

    }

    /**
     * Check if we are on the Blue alliance.
     */
    public static boolean isBlueAlliance()
    {
        var alliance = DriverStation.getAlliance();
        return alliance.isPresent() && alliance.get() == Alliance.Blue;
    }

    /**
     * Check if we are on the Red alliance.
     */
    public static boolean isRedAlliance()
    {
        var alliance = DriverStation.getAlliance();
        return alliance.isPresent() && alliance.get() == Alliance.Red;
    }

    /**
     * Get the HUB AprilTag IDs for our alliance. Use these IDs to filter Limelight
     * detections for shooting. We shoot at OUR OWN hub (not opponent's).
     */
    public static List<Double> getOurHubTagIds()
    {
        var alliance = DriverStation.getAlliance();
        if (alliance.isEmpty()) return kAllHubTagIds;
        if (isRedAlliance())
        {
            return ShooterConstants.RED_HUB_TAG_IDS;
        }
        return ShooterConstants.BLUE_HUB_TAG_IDS;
    }

    private static int getAllianceShift(double timeRemaining)
    {
        if (timeRemaining > kTransitionShiftEndTimeSecs) return 0; // Transition Shift
        if (timeRemaining > kShift1EndTimeSecs) return 1;
        if (timeRemaining > kShift2EndTimeSecs) return 2;
        if (timeRemaining > kShift3EndTimeSecs) return 3;
        if (timeRemaining > kShift4EndTimeSecs) return 4;
        return 5; // End Game
    }

    private static Alliance getInactiveFirstAlliance(String gameData)
    {
        if (gameData == null || gameData.isEmpty()) return null;

        char inactiveFirstAlliance = Character.toUpperCase(gameData.charAt(0));
        if (inactiveFirstAlliance == 'R') return Alliance.Red;
        if (inactiveFirstAlliance == 'B') return Alliance.Blue;
        return null;
    }
}
