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
