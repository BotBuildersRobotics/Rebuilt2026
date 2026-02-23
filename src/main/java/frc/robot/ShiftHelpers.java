package frc.robot;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringEntry;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public final class ShiftHelpers {
    private ShiftHelpers() {}

    public enum HubState {
        BOTH_ACTIVE,
        RED_ACTIVE,
        BLUE_ACTIVE,
        INVALID
    }

    private static final double[] SHIFT_BOUNDARIES = { 130.0, 105.0, 80.0, 55.0, 30.0 };
    private static final double SHIFT_LEAD_SECONDS = 2.0;
    private static final double TIME_EPS = 0;

    private static final StringEntry overrideEntry =
        NetworkTableInstance.getDefault()
            .getTable("SmartDashboard")
            .getStringTopic("Rebuilt/GameDataOverride")
            .getEntry("");

    static {
        overrideEntry.setDefault("");
    }

    private static volatile char latchedFirstInactive = '\0';

    public static String getGameData() {
        String raw;
        if (DriverStation.isFMSAttached()) {
            String ds = DriverStation.getGameSpecificMessage();
            raw = (ds == null) ? "" : ds.trim();
        } else {
            raw = overrideEntry.get().trim();
        }
        return raw;
    }

    public static char getFirstInactiveAllianceChar() {
        String data = getGameData();
        if (data.isEmpty()) return '\0';

        char c = Character.toUpperCase(data.charAt(0));
        return (c == 'R' || c == 'B') ? c : '\0';
    }

    private static char getLatchedFirstInactiveAllianceChar() {
        char live = getFirstInactiveAllianceChar();
        if (live != '\0') latchedFirstInactive = live;
        return latchedFirstInactive;
    }

    public static int getTeleopShiftIndex() {
        double t = Timer.getMatchTime();
        if (t <= 0) return 5;

        double tEff = t - SHIFT_LEAD_SECONDS;

        if (tEff > SHIFT_BOUNDARIES[0] + TIME_EPS) return 0;
        if (tEff > SHIFT_BOUNDARIES[1] + TIME_EPS) return 1;
        if (tEff > SHIFT_BOUNDARIES[2] + TIME_EPS) return 2;
        if (tEff > SHIFT_BOUNDARIES[3] + TIME_EPS) return 3;
        if (tEff > SHIFT_BOUNDARIES[4] + TIME_EPS) return 4;
        return 5;
    }

    private static boolean isHubActiveForAlliance(DriverStation.Alliance alliance) {
        if (DriverStation.isAutonomousEnabled()) return true;

        if (!DriverStation.isTeleopEnabled()) return false;

        int shift = getTeleopShiftIndex();

        if (shift == 0 || shift == 5) return true;

        char firstInactive = getLatchedFirstInactiveAllianceChar();
        if (firstInactive == '\0') return true;

        boolean isFirstInactiveAlliance =
            (firstInactive == 'R' && alliance == DriverStation.Alliance.Red) ||
            (firstInactive == 'B' && alliance == DriverStation.Alliance.Blue);

        boolean firstInactiveHubActive = (shift == 2 || shift == 4);

        return isFirstInactiveAlliance == firstInactiveHubActive;
    }

    private static boolean isOurHubActive() {
        if (DriverStation.isAutonomousEnabled()) return true;

        var ourAllianceOpt = DriverStation.getAlliance();
        if (ourAllianceOpt.isEmpty()) return false;

        boolean active = isHubActiveForAlliance(ourAllianceOpt.get());
        return active;
    }

    public static Trigger hubActive() {
        return new Trigger(ShiftHelpers::isOurHubActive);
    }

    public static HubState getCurrentHubState() {
        if (DriverStation.isAutonomousEnabled()) return HubState.BOTH_ACTIVE;
        if (!DriverStation.isTeleopEnabled()) return HubState.INVALID;

        int shift = getTeleopShiftIndex();
        if (shift == 0 || shift == 5) return HubState.BOTH_ACTIVE;

        char firstInactive = getLatchedFirstInactiveAllianceChar();
        if (firstInactive == '\0') return HubState.BOTH_ACTIVE;

        boolean firstInactiveHubActive = (shift == 2 || shift == 4);

        DriverStation.Alliance active =
            firstInactiveHubActive
                ? (firstInactive == 'R' ? DriverStation.Alliance.Red : DriverStation.Alliance.Blue)
                : (firstInactive == 'R' ? DriverStation.Alliance.Blue : DriverStation.Alliance.Red);

        return (active == DriverStation.Alliance.Red) ? HubState.RED_ACTIVE : HubState.BLUE_ACTIVE;
    }

    private static boolean teleopShiftChangeImminent(double seconds) {
        if (!DriverStation.isTeleopEnabled()) return false;

        double t = Timer.getMatchTime();
        if (t < 0) return false;

        for (double boundary : SHIFT_BOUNDARIES) {
            double dt = t - boundary;
            if (dt >= 0 && dt <= seconds) return true;
        }
        return false;
    }

    public static Trigger shiftAboutToChange(double seconds) {
        return new Trigger(() -> teleopShiftChangeImminent(seconds));
    }

    public static int timeLeftInShiftSeconds() {
        if (DriverStation.isAutonomousEnabled()) return -1;
        if (!DriverStation.isTeleopEnabled()) return 0;

        double t = Timer.getMatchTime();
        if (t <= 0) return 0;

        int shift = getTeleopShiftIndex();

        if (shift == 0) {
            double endAt = SHIFT_BOUNDARIES[0] + SHIFT_LEAD_SECONDS;
            return (int) Math.max(0, Math.ceil(t - endAt));
        }

        double tEff = t - SHIFT_LEAD_SECONDS;

        if (shift == 5) return (int) Math.max(0, Math.ceil(tEff));

        double endBoundary = SHIFT_BOUNDARIES[shift];
        return (int) Math.max(0, Math.ceil(tEff - endBoundary));
    }

    public static boolean isRedShiftActive() {
        return getCurrentHubState() == HubState.RED_ACTIVE;
    }

    public static boolean isBlueShiftActive() {
        return getCurrentHubState() == HubState.BLUE_ACTIVE;
    }

    public static boolean isBothHubsActive() {
        return getCurrentHubState() == HubState.BOTH_ACTIVE;
    }

    public static Trigger redShiftActive() {
        return new Trigger(ShiftHelpers::isRedShiftActive);
    }

    public static Trigger blueShiftActive() {
        return new Trigger(ShiftHelpers::isBlueShiftActive);
    }

    public static Trigger bothHubsActive() {
        return new Trigger(ShiftHelpers::isBothHubsActive);
    }

    public static boolean isRedHubActive() {
        return isHubActiveForAlliance(DriverStation.Alliance.Red);
    }

    public static boolean isBlueHubActive() {
        return isHubActiveForAlliance(DriverStation.Alliance.Blue);
    }
}
