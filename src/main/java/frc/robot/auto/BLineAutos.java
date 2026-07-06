// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.auto;

import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;

import java.io.File;
import java.io.FileReader;
import java.util.Arrays;
import java.util.LinkedHashMap;
import java.util.Map;

import org.json.simple.JSONArray;
import org.json.simple.JSONObject;
import org.json.simple.parser.JSONParser;

import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

import frc.robot.lib.BLine.BLineCommands;
import frc.robot.lib.BLine.FollowPath;
import frc.robot.lib.BLine.JsonUtils;
import frc.robot.lib.BLine.Path;
import frc.robot.subsystems.SuperSystem;
import frc.robot.subsystems.drive.CommandSwerveDrivetrain;
import frc.robot.subsystems.drive.DriveConstants;
import frc.robot.subsystems.drive.DriveSubsystem;

/**
 * BLine autonomous integration.
 *
 * <p>Runs alongside PathPlanner as a second path-following engine. BLine drives the same swerve
 * base through the same four hooks PathPlanner uses (pose supplier, pose reset, robot-relative
 * speeds supplier, robot-relative speeds consumer), so nothing about the PathPlanner setup changes.
 *
 * <p>BLine paths are authored in the BLine GUI and exported to {@code src/main/deploy/autos/paths/}
 * (they land in {@code <deployDir>/autos/paths/} on the roboRIO). Global constraints come from
 * {@code src/main/deploy/autos/config.json}; when a path is loaded from a file BLine reads that file
 * directly, so it must be present. The in-code {@link Path#setDefaultGlobalConstraints} call below is
 * only a fallback for paths built from waypoints in Java.
 *
 * <p>Composed autos (drive + score/intake) are built in {@link #defineAutos()} by chaining
 * {@link #path(String)} with the existing {@link SuperSystem} actions — the same actions PathPlanner
 * invokes through {@code NamedCommands}. Actions that must happen <em>during</em> a path should use
 * BLine event triggers ({@link FollowPath#registerEventTrigger}) instead of between-path steps.
 */
public class BLineAutos {

    private static BLineAutos instance;

    public static BLineAutos getInstance() {
        if (instance == null) {
            instance = new BLineAutos();
        }
        return instance;
    }

    private final FollowPath.Builder builder;

    private BLineAutos() {
        // Fallback global constraints for Java-built paths. File-loaded paths use deploy/autos/config.json.
        Path.setDefaultGlobalConstraints(new Path.DefaultGlobalConstraints(
                DriveConstants.kMaxSpeed.in(MetersPerSecond),            // maxVelocity m/s
                DriveConstants.kMaxAcceleration.in(MetersPerSecondPerSecond), // maxAccel m/s^2
                DriveConstants.kMaxAngularRate.in(DegreesPerSecond),     // maxVelocity deg/s
                720.0,                                                   // maxAccel deg/s^2
                0.03,                                                    // end translation tolerance (m)
                2.0,                                                     // end rotation tolerance (deg)
                0.25));                                                  // intermediate handoff radius (m)

        CommandSwerveDrivetrain drivetrain = DriveSubsystem.mInstance.getGeneratedDrive();

        // PID gains are BLine's own controller model — do NOT copy PathPlanner's (10 / 7) gains.
        // Start from the BLine doc defaults and tune translation / rotation / cross-track on the field.
        builder = new FollowPath.Builder(
                        drivetrain,                          // Subsystem requirement
                        drivetrain.getPose(),                // Supplier<Pose2d> (getPose() returns a supplier)
                        () -> drivetrain.getState().Speeds,  // robot-relative ChassisSpeeds supplier
                        drivetrain::driveRobotRelative,      // robot-relative ChassisSpeeds consumer
                        new PIDController(5.0, 0.0, 0.0),    // translation
                        new PIDController(3.0, 0.0, 0.0),    // rotation
                        new PIDController(2.0, 0.0, 0.0))    // cross-track
                .withDefaultShouldFlip();                    // flip for red alliance (matches PathPlanner)

        registerEventTriggers();
    }

    /**
     * Binds the {@code lib_key} event markers embedded in the BLine path JSONs to robot actions.
     *
     * <p>This is a static, global registry ({@link FollowPath#registerEventTrigger}) shared by every
     * BLine path command, so it only needs to run once. Each key here must match a marker's
     * {@code lib_key} in the GUI. The actions are the same {@link SuperSystem} commands PathPlanner
     * invokes through {@code NamedCommands}.
     *
     * <p>The {@code Command} form schedules the action on the {@link edu.wpi.first.wpilibj2.command.CommandScheduler}
     * when the marker is reached; BLine does not auto-cancel it when the path ends. None of these require
     * the drive subsystem, so they won't fight the path follower. If you later add a marker action that
     * requires drive (or a subsystem also held by the surrounding sequence), compose the auto with
     * {@link frc.robot.lib.BLine.BLineCommands} so requirements aren't held for the whole routine.
     */
    private void registerEventTriggers() {
        // --- Intake markers ---
        FollowPath.registerEventTrigger("intakeDeploy", SuperSystem.mInstance.DeployIntake());
        FollowPath.registerEventTrigger("intakeOn", SuperSystem.mInstance.Intake());
        FollowPath.registerEventTrigger("intakeOff", SuperSystem.mInstance.idleIntakes());

        // --- Shooter / feed markers ---
        FollowPath.registerEventTrigger("shoot", SuperSystem.mInstance.ShootAuto());
        // NOTE: "shootPause" and "shootStop" both stop the feed (idleShooter) by default. If you intend
        // different behavior — e.g. shootStop should also spin the flywheel down — retarget them here.
        FollowPath.registerEventTrigger("shootPause", SuperSystem.mInstance.idleShooter());
        FollowPath.registerEventTrigger("shootStop", SuperSystem.mInstance.idleShooter());
    }

    /**
     * Follows a single BLine path file, seeding odometry to the path's start pose (reset = true).
     * Use for standalone path tests and for the <em>first</em> segment of a multi-segment auto.
     *
     * @param name Path file name (without {@code .json}) in {@code deploy/autos/paths/}
     */
    public Command path(String name) {
        return path(name, true);
    }

    /**
     * Follows a single BLine path file from {@code deploy/autos/paths/<name>.json}.
     *
     * <p>When {@code resetPoseAtStart} is false the command trusts the existing odometry (gyro + vision)
     * instead of snapping the pose to the segment's nominal start — this matches PathPlanner's
     * {@code "resetOdom": false} and is what you want for the 2nd/3rd segments of an auto, so a mid-auto
     * pose jump can't drive the robot into a trench wall. Only the first segment (or a standalone test)
     * should reset.
     *
     * <p>Loading is guarded: a missing or malformed file logs an error and yields a no-op command rather
     * than crashing robot initialization (mirrors how the PathPlanner AutoBuilder config is guarded).
     * The pose-reset toggle mutates the shared builder, which is safe because all paths are built once,
     * synchronously, at robot startup.
     *
     * @param name Path file name without the {@code .json} extension
     * @param resetPoseAtStart true to seed odometry to the path start; false to trust existing odometry
     * @return A follow command for the path, or a no-op command if it could not be loaded
     */
    public Command path(String name, boolean resetPoseAtStart) {
        try {
            if (resetPoseAtStart) {
                builder.withPoseReset(DriveSubsystem.mInstance.getGeneratedDrive()::resetPose);
            } else {
                builder.withPoseReset(pose -> {});
            }
            return builder.build(new Path(name)).withName("BLinePath:" + name);
        } catch (Exception ex) {
            DriverStation.reportError("BLine: failed to load path '" + name + "'", ex.getStackTrace());
            return Commands.none().withName("BLinePath(missing):" + name);
        }
    }

    /** Directory holding JSON auto routines: {@code <deployDir>/autos/routines/*.json}. */
    private static final File ROUTINES_DIR = new File(JsonUtils.PROJECT_ROOT, "routines");

    /**
     * Optional Java-defined autos. Prefer JSON routines (see {@link #loadRoutines}) so autos can be
     * managed at competition without recompiling. Add an entry here only for logic that can't be
     * expressed as a routine tree.
     */
    private Map<String, Command> defineAutos() {
        Map<String, Command> autos = new LinkedHashMap<>();
        // (empty) — TrenchBump - Left now lives as deploy/autos/routines/TrenchBump - Left.json
        return autos;
    }

    /**
     * Loads competition-managed autos from {@code deploy/autos/routines/*.json} and adds them to the
     * chooser. The routine schema is intentionally the <b>same as a PathPlanner {@code .auto} file</b>
     * — a command tree of {@code sequential} / {@code parallel} / {@code race} / {@code deadline} /
     * {@code wait} / {@code named} / {@code path} nodes — so you can copy a PathPlanner auto straight
     * over (just point {@code pathName} at BLine path files) and tweak it at an event without touching
     * Java. {@code named} steps resolve through the very same {@link NamedCommands} registry PathPlanner
     * uses, so the blocking "DeployIntake before the trench" step behaves identically.
     *
     * <p>Path following uses {@link #path}. Odometry reset honors the routine's top-level
     * {@code "resetOdom"} (default true): when true, only the <em>first</em> {@code path} node seeds the
     * pose to its start; later paths trust existing odometry (gyro + vision) so there is no mid-auto pose
     * snap. Set it false to never reset (matches a PathPlanner auto with {@code "resetOdom": false}).
     *
     * <p>Each file is guarded: a malformed routine logs an error and is skipped rather than crashing
     * robot init.
     */
    private void loadRoutines(SendableChooser<Command> chooser) {
        File[] files = ROUTINES_DIR.listFiles((dir, fileName) -> fileName.toLowerCase().endsWith(".json"));
        if (files == null) {
            return;
        }
        Arrays.sort(files);
        for (File file : files) {
            String name = file.getName().substring(0, file.getName().length() - ".json".length());
            try (FileReader reader = new FileReader(file)) {
                JSONObject root = (JSONObject) new JSONParser().parse(reader);
                // resetOdom defaults to true; false only if explicitly set false.
                boolean[] resetPending = { !Boolean.FALSE.equals(root.get("resetOdom")) };
                Command cmd = buildCommand((JSONObject) root.get("command"), resetPending)
                        .withName("BLine:" + name);
                chooser.addOption("BLine: " + name, cmd);
            } catch (Exception ex) {
                DriverStation.reportError("BLine: failed to load routine '" + name + "'", ex.getStackTrace());
            }
        }
    }

    /**
     * Recursively builds a WPILib command from a routine node (PathPlanner {@code .auto} command schema).
     * Compositions use {@link BLineCommands} so child requirements are proxied and not held for the whole
     * routine. {@code resetPending[0]} carries the one-shot "first path reseeds odometry" flag.
     */
    private Command buildCommand(JSONObject node, boolean[] resetPending) {
        if (node == null) {
            return Commands.none();
        }
        String type = (String) node.get("type");
        JSONObject data = node.get("data") instanceof JSONObject ? (JSONObject) node.get("data") : node;
        switch (type == null ? "" : type) {
            case "sequential":
                return BLineCommands.sequence(buildChildren(data, resetPending));
            case "parallel":
                return BLineCommands.parallel(buildChildren(data, resetPending));
            case "race":
                return BLineCommands.race(buildChildren(data, resetPending));
            case "deadline": {
                Command[] cmds = buildChildren(data, resetPending);
                if (cmds.length == 0) {
                    return Commands.none();
                }
                return BLineCommands.deadline(cmds[0], Arrays.copyOfRange(cmds, 1, cmds.length));
            }
            case "wait": {
                Object t = data.get("waitTime");
                return Commands.waitSeconds(t instanceof Number ? ((Number) t).doubleValue() : 0.0);
            }
            case "named":
                return namedCommand((String) data.get("name"));
            case "path": {
                boolean reset = resetPending[0];
                resetPending[0] = false; // only the first path in document order reseeds odometry
                return path((String) data.get("pathName"), reset);
            }
            default:
                DriverStation.reportWarning("BLine routine: unknown command type '" + type + "'", false);
                return Commands.none();
        }
    }

    private Command[] buildChildren(JSONObject data, boolean[] resetPending) {
        Object commands = data.get("commands");
        if (!(commands instanceof JSONArray)) {
            return new Command[0];
        }
        JSONArray arr = (JSONArray) commands;
        Command[] out = new Command[arr.size()];
        for (int i = 0; i < arr.size(); i++) {
            out[i] = buildCommand((JSONObject) arr.get(i), resetPending);
        }
        return out;
    }

    /**
     * Resolves a {@code named} routine step through PathPlanner's {@link NamedCommands} registry — the
     * same registry the PathPlanner autos use — so both engines share one source of truth for actions.
     */
    private Command namedCommand(String name) {
        if (name != null && NamedCommands.hasCommand(name)) {
            return NamedCommands.getCommand(name);
        }
        DriverStation.reportWarning("BLine routine: no NamedCommand registered for '" + name + "'", false);
        return Commands.none();
    }

    /**
     * Adds BLine options to the shared auto chooser, prefixed with {@code "BLine: "} so they sit
     * alongside the PathPlanner autos. Registers, in order: any Java-defined autos ({@link #defineAutos}),
     * the JSON routines ({@link #loadRoutines}), and every raw path JSON in {@code deploy/autos/paths/} as
     * a drive-only {@code [path]} option (handy for testing path following in isolation).
     */
    public void registerBLineAutos(SendableChooser<Command> chooser) {
        for (Map.Entry<String, Command> entry : defineAutos().entrySet()) {
            chooser.addOption("BLine: " + entry.getKey(), entry.getValue());
        }

        loadRoutines(chooser);

        File pathsDir = new File(JsonUtils.PROJECT_ROOT, "paths");
        File[] files = pathsDir.listFiles((dir, fileName) -> fileName.toLowerCase().endsWith(".json"));
        if (files != null) {
            for (File file : files) {
                String name = file.getName().substring(0, file.getName().length() - ".json".length());
                chooser.addOption("BLine: [path] " + name, path(name));
            }
        }
    }
}
