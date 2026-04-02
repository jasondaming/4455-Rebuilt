// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

import frc.robot.subsystems.CANFuelSubsystem;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.CommandSwerveDrivetrain;

import static frc.robot.Constants.FuelConstants.*;
import static frc.robot.Constants.ClimbConstants.*;
import static frc.robot.Constants.AutoConstants.*;

/**
 * Factory for autonomous named commands used by PathPlanner.
 *
 * <p>Call {@link #registerNamedCommands} once, before building the auto chooser.
 *
 * <p>Named commands available for use in .auto files:
 * <ul>
 *   <li>"Shoot"     – near shot (~3 ft from hub): spin-up then feed</li>
 *   <li>"ShootFar"  – far shot (from tower): higher launcher power, longer feed</li>
 *   <li>"HookOpen"  – run climber UP for {@code AUTO_HOOK_OPEN_SECONDS} to open hooks</li>
 *   <li>"ClimbDown" – run climber DOWN for {@code AUTO_HOOK_CLOSE_SECONDS} to hang</li>
 * </ul>
 */
public class Autos {

    /**
     * Registers all named commands with PathPlanner. Must be called before
     * {@code AutoBuilder.buildAutoChooser()}.
     */
    public static void registerNamedCommands(CANFuelSubsystem fuel, ClimberSubsystem climber) {
        NamedCommands.registerCommand("Shoot",     buildShootCommand(fuel));
        NamedCommands.registerCommand("ShootFar",  buildShootFarCommand(fuel));
        NamedCommands.registerCommand("HookOpen",  buildHookOpenCommand(climber));
        NamedCommands.registerCommand("ClimbDown", buildClimbDownCommand(climber));
        NamedCommands.registerCommand("SpinUp",    buildSpinUpCommand(fuel));
        NamedCommands.registerCommand("ShootFeed", buildShootFeedCommand(fuel));
    }

    // -----------------------------------------------------------------------
    // Command builders
    // -----------------------------------------------------------------------

    /**
     * Near shoot sequence (~3 ft from hub):
     * spin-up (0.75 s) then feed ({@code AUTO_FEED_NEAR_SECONDS}), then stop.
     */
    private static Command buildShootCommand(CANFuelSubsystem fuel) {
        return Commands.sequence(
            Commands.run(() -> {
                fuel.setIntakeLauncherRoller(-0.8 * LAUNCHING_LAUNCHER_PERCENT);
                fuel.setFeederRoller(0.8 * INDEXER_SPIN_UP_PRE_LAUNCH_PERCENT);
            }, fuel)
            .withTimeout(SPIN_UP_SECONDS)
            .beforeStarting(Commands.runOnce(() -> {
                System.out.println("[Auto][Shoot] SpinUp start");
                SmartDashboard.putString("Auto/Phase", "SpinUp");
            })),

            Commands.run(() -> {
                fuel.setIntakeLauncherRoller(-0.8 * LAUNCHING_LAUNCHER_PERCENT);
                fuel.setFeederRoller(0.8 * INDEXER_LAUNCHING_PERCENT);
            }, fuel)
            .withTimeout(AUTO_FEED_NEAR_SECONDS)
            .beforeStarting(Commands.runOnce(() -> {
                System.out.println("[Auto][Shoot] Feeding");
                SmartDashboard.putString("Auto/Phase", "Feeding");
            })),

            Commands.runOnce(() -> {
                fuel.stop();
                System.out.println("[Auto][Shoot] Done");
                SmartDashboard.putString("Auto/Phase", "ShootDone");
            }, fuel)
        ).withName("Shoot");
    }

    /**
     * Far shoot sequence (from tower position):
     * higher launcher power ({@code AUTO_LAUNCHER_FAR_MULT}) and longer feed time.
     */
    private static Command buildShootFarCommand(CANFuelSubsystem fuel) {
        return Commands.sequence(
            Commands.run(() -> {
                fuel.setIntakeLauncherRoller(-AUTO_LAUNCHER_FAR_MULT * LAUNCHING_LAUNCHER_PERCENT);
                fuel.setFeederRoller(AUTO_LAUNCHER_FAR_MULT * INDEXER_SPIN_UP_PRE_LAUNCH_PERCENT);
            }, fuel)
            .withTimeout(SPIN_UP_SECONDS)
            .beforeStarting(Commands.runOnce(() -> {
                System.out.println("[Auto][ShootFar] SpinUp start");
                SmartDashboard.putString("Auto/Phase", "SpinUpFar");
            })),

            Commands.run(() -> {
                fuel.setIntakeLauncherRoller(-AUTO_LAUNCHER_FAR_MULT * LAUNCHING_LAUNCHER_PERCENT);
                fuel.setFeederRoller(AUTO_LAUNCHER_FAR_MULT * INDEXER_LAUNCHING_PERCENT);
            }, fuel)
            .withTimeout(AUTO_FEED_FAR_SECONDS)
            .beforeStarting(Commands.runOnce(() -> {
                System.out.println("[Auto][ShootFar] Feeding");
                SmartDashboard.putString("Auto/Phase", "FeedingFar");
            })),

            Commands.runOnce(() -> {
                fuel.stop();
                System.out.println("[Auto][ShootFar] Done");
                SmartDashboard.putString("Auto/Phase", "ShootFarDone");
            }, fuel)
        ).withName("ShootFar");
    }

    /**
     * Opens hooks by running the climber UP for {@code AUTO_HOOK_OPEN_SECONDS}.
     * Runs in parallel with the initial drive segment.
     */
    private static Command buildHookOpenCommand(ClimberSubsystem climber) {
        return Commands.sequence(
            Commands.runOnce(() -> {
                System.out.println("[Auto][HookOpen] Opening hooks");
                SmartDashboard.putString("Auto/Phase", "HookOpen");
            }),
            Commands.run(() -> climber.setClimber(CLIMBER_MOTOR_UP_PERCENT), climber)
                .withTimeout(AUTO_HOOK_OPEN_SECONDS)
                .finallyDo(() -> {
                    climber.stop();
                    System.out.println("[Auto][HookOpen] Done");
                })
        ).withName("HookOpen");
    }

    /**
     * Spins up the launcher without feeding. Run in parallel with a drive path so
     * the launcher is already at speed when the path ends and ShootFeed fires.
     */
    private static Command buildSpinUpCommand(CANFuelSubsystem fuel) {
        return Commands.sequence(
            Commands.runOnce(() -> {
                System.out.println("[Auto][SpinUp] Spinning up");
                SmartDashboard.putString("Auto/Phase", "SpinUp");
            }),
            Commands.run(() -> {
                fuel.setIntakeLauncherRoller(-0.8 * LAUNCHING_LAUNCHER_PERCENT);
                fuel.setFeederRoller(0.8 * INDEXER_SPIN_UP_PRE_LAUNCH_PERCENT);
            }, fuel)
            .withTimeout(SPIN_UP_SECONDS)
            .andThen(Commands.runOnce(() -> System.out.println("[Auto][SpinUp] Done — launcher staying on")))
        ).withName("SpinUp");
    }

    /**
     * Feed-only shoot: launcher already spun up, just run the feeder.
     * Pair with {@code SpinUp} run in parallel during the preceding drive.
     */
    private static Command buildShootFeedCommand(CANFuelSubsystem fuel) {
        return Commands.sequence(
            Commands.run(() -> {
                fuel.setIntakeLauncherRoller(-0.8 * LAUNCHING_LAUNCHER_PERCENT);
                fuel.setFeederRoller(0.8 * INDEXER_LAUNCHING_PERCENT);
            }, fuel)
            .withTimeout(AUTO_FEED_NEAR_SECONDS)
            .beforeStarting(Commands.runOnce(() -> {
                System.out.println("[Auto][ShootFeed] Feeding");
                SmartDashboard.putString("Auto/Phase", "ShootFeed");
            })),
            Commands.runOnce(() -> {
                fuel.stop();
                System.out.println("[Auto][ShootFeed] Done");
                SmartDashboard.putString("Auto/Phase", "ShootFeedDone");
            }, fuel)
        ).withName("ShootFeed");
    }

    /**
     * Everybot fallback auto — no PathPlanner, just timed robot-centric drives.
     *
     * <ol>
     *   <li>parallel: drive forward {@code EVERYBOT_DRIVE1_SECONDS}, open hooks, spin up launcher</li>
     *   <li>feed ({@code AUTO_FEED_NEAR_SECONDS})</li>
     *   <li>drive forward {@code EVERYBOT_DRIVE2_SECONDS} until frame hits tower post</li>
     *   <li>ClimbDown {@code AUTO_HOOK_CLOSE_SECONDS}</li>
     * </ol>
     *
     * "Forward" = robot-centric -X (front of robot faces tower).
     */
    public static Command buildEverbotCommand(CANFuelSubsystem fuel, ClimberSubsystem climber,
            CommandSwerveDrivetrain drivetrain) {
        final SwerveRequest.RobotCentric driveReq = new SwerveRequest.RobotCentric();
        final SwerveRequest.SwerveDriveBrake brakeReq = new SwerveRequest.SwerveDriveBrake();

        Command drive1 = drivetrain.applyRequest(
                () -> driveReq.withVelocityX(EVERYBOT_DRIVE_SPEED).withVelocityY(0).withRotationalRate(0))
            .withTimeout(EVERYBOT_DRIVE1_SECONDS)
            .beforeStarting(Commands.runOnce(() -> {
                System.out.println("[Auto][Everybot] Drive 1 start");
                SmartDashboard.putString("Auto/Phase", "Everybot_Drive1");
            }));

        Command drive2 = drivetrain.applyRequest(
                () -> driveReq.withVelocityX(EVERYBOT_DRIVE_SPEED).withVelocityY(0).withRotationalRate(0))
            .withTimeout(EVERYBOT_DRIVE2_SECONDS)
            .beforeStarting(Commands.runOnce(() -> {
                System.out.println("[Auto][Everybot] Drive 2 start");
                SmartDashboard.putString("Auto/Phase", "Everybot_Drive2");
            }));

        // drive1 is the deadline — HookOpen and SpinUp are cancelled when drive1 ends.
        // During ShootFeed and ClimbDown the drivetrain is explicitly braked so the
        // default joystick command cannot take over and drive the robot.
        return Commands.sequence(
            Commands.deadline(drive1, buildHookOpenCommand(climber), buildSpinUpCommand(fuel)),
            Commands.deadline(buildShootFeedCommand(fuel),   drivetrain.applyRequest(() -> brakeReq)),
            drive2,
            Commands.deadline(buildClimbDownCommand(climber), drivetrain.applyRequest(() -> brakeReq))
        ).withName("Everybot");
    }

    /**
     * Hangs on the tower by running the climber DOWN for {@code AUTO_HOOK_CLOSE_SECONDS}.
     */
    private static Command buildClimbDownCommand(ClimberSubsystem climber) {
        return Commands.sequence(
            Commands.runOnce(() -> {
                System.out.println("[Auto][ClimbDown] Hanging");
                SmartDashboard.putString("Auto/Phase", "ClimbDown");
            }),
            Commands.run(() -> climber.setClimber(CLIMBER_MOTOR_DOWN_PERCENT), climber)
                .withTimeout(AUTO_HOOK_CLOSE_SECONDS)
                .finallyDo(() -> {
                    climber.stop();
                    System.out.println("[Auto][ClimbDown] Done");
                    SmartDashboard.putString("Auto/Phase", "Hung");
                })
        ).withName("ClimbDown");
    }
}
