package frc.robot;

public final class LoggingConfig
{
    private LoggingConfig()
    {
    }

    /**
     * Mirrors NetworkTables data to a WPILib .wpilog file so AdvantageScope can
     * inspect match telemetry after the fact.
     */
    public static final boolean ENABLE_WPILIB_DATA_LOG  = true;
    /**
     * Phoenix SignalLogger writes CTRE .hoot logs. Leave this disabled unless we
     * intentionally want Phoenix replay/SysId logging.
     */
    public static final boolean ENABLE_CTRE_SIGNAL_LOG  = false;
    /**
     * Loads a .hoot file for Phoenix replay. This is separate from live logging so
     * we can keep replay support off by default without changing the path.
     */
    public static final boolean ENABLE_CTRE_HOOT_REPLAY = false;
    public static final String  CTRE_HOOT_REPLAY_LOG    = "./logs/example.hoot";
}
