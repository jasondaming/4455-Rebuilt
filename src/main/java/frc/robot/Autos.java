// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

import frc.robot.subsystems.CANFuelSubsystem;
import frc.robot.subsystems.ClimberSubsystem;

import static frc.robot.Constants.FuelConstants.*;
import static frc.robot.Constants.ClimbConstants.*;

/**
 * Factory for autonomous named commands used by PathPlanner.
 *
 * <p>Call {@link #registerNamedCommands} once, before building the auto chooser.
 *
 * <p>Named commands available for use in .auto files:
 * <ul>
 *   <li>"Shoot"   – spin-up then feed game piece (~2.25 s total)</li>
 *   <li>"ClimbUp" – run climber upward until auto ends</li>
 * </ul>
 *
 * <p>TUNING NOTES:
 * <ul>
 *   <li>Spin-up duration: {@code Constants.FuelConstants.SPIN_UP_SECONDS} (currently 0.75 s)</li>
 *   <li>Feed duration: {@code AUTO_FEED_SECONDS} constant below (currently 1.5 s)</li>
 *   <li>Motor percentages: pulled from FuelConstants / ClimbConstants</li>
 * </ul>
 */
public class Autos {

    /**
     * Duration (seconds) to run the feeder during the auto shoot sequence.
     * Increase if the game piece doesn't fully exit; decrease to save time.
     */
    private static final double AUTO_FEED_SECONDS = 1.5;

    /**
     * Registers all named commands with PathPlanner. Must be called before
     * {@code AutoBuilder.buildAutoChooser()}.
     */
    public static void registerNamedCommands(CANFuelSubsystem fuel, ClimberSubsystem climber) {
        NamedCommands.registerCommand("Shoot",   buildShootCommand(fuel));
        NamedCommands.registerCommand("ClimbUp", buildClimbUpCommand(climber));
    }

    // -----------------------------------------------------------------------
    // Command builders
    // -----------------------------------------------------------------------

    /**
     * Full shoot sequence:
     * <ol>
     *   <li>Spin-up: launchers at speed, feeder slightly reversed (SPIN_UP_SECONDS)</li>
     *   <li>Feed: launchers + feeder full send (AUTO_FEED_SECONDS)</li>
     *   <li>Stop all rollers</li>
     * </ol>
     *
     * <p>SmartDashboard key "Auto/Phase" is updated at each transition for easy
     * dashboard monitoring. System.out lines appear in the Rio log / console.
     */
    private static Command buildShootCommand(CANFuelSubsystem fuel) {
        return Commands.sequence(
            // ── Phase 1: spin-up ────────────────────────────────────────────
            Commands.run(() -> {
                fuel.setIntakeLauncherRoller(-0.8 * LAUNCHING_LAUNCHER_PERCENT);
                fuel.setFeederRoller(0.8 * INDEXER_SPIN_UP_PRE_LAUNCH_PERCENT);
            }, fuel)
            .withTimeout(SPIN_UP_SECONDS)
            .beforeStarting(Commands.runOnce(() -> {
                System.out.println("[Auto][Shoot] SpinUp start");
                SmartDashboard.putString("Auto/Phase", "SpinUp");
            })),

            // ── Phase 2: feed ────────────────────────────────────────────────
            Commands.run(() -> {
                fuel.setIntakeLauncherRoller(-0.8 * LAUNCHING_LAUNCHER_PERCENT);
                fuel.setFeederRoller(0.8 * INDEXER_LAUNCHING_PERCENT);
            }, fuel)
            .withTimeout(AUTO_FEED_SECONDS)
            .beforeStarting(Commands.runOnce(() -> {
                System.out.println("[Auto][Shoot] Feeding");
                SmartDashboard.putString("Auto/Phase", "Feeding");
            })),

            // ── Stop ─────────────────────────────────────────────────────────
            Commands.runOnce(() -> {
                fuel.stop();
                System.out.println("[Auto][Shoot] Done");
                SmartDashboard.putString("Auto/Phase", "ShootDone");
            }, fuel)
        ).withName("Shoot");
    }

    /**
     * Runs the climber upward until the command is interrupted (i.e., auto ends).
     * The climber stops when the command is cancelled.
     */
    private static Command buildClimbUpCommand(ClimberSubsystem climber) {
        return Commands.sequence(
            Commands.runOnce(() -> {
                System.out.println("[Auto][Climb] Starting climb");
                SmartDashboard.putString("Auto/Phase", "ClimbUp");
            }),
            Commands.run(() -> climber.setClimber(CLIMBER_MOTOR_UP_PERCENT), climber)
                .finallyDo(() -> {
                    climber.stop();
                    System.out.println("[Auto][Climb] Stopped");
                })
        ).withName("ClimbUp");
    }
}
