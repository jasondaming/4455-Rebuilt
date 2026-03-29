// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.FollowPathCommand;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

import static frc.robot.Constants.OperatorConstants.*;
import static frc.robot.Constants.FuelConstants.*;
import static frc.robot.Constants.ClimbConstants.*;

import frc.robot.subsystems.CANFuelSubsystem;
import frc.robot.subsystems.ClimberSubsystem;

import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class RobotContainer {
    private double MaxSpeed = 1.0 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond);

    private final CANFuelSubsystem fuelSubsystem = new CANFuelSubsystem();
    private final ClimberSubsystem climberSubsystem = new ClimberSubsystem();

    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1)
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
    private final SwerveRequest.RobotCentric forwardStraight = new SwerveRequest.RobotCentric()
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final CommandXboxController driverController = new CommandXboxController(DRIVER_CONTROLLER_PORT);
    private final CommandXboxController operatorController = new CommandXboxController(OPERATOR_CONTROLLER_PORT);

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    private double turtlemode = 1.0;

    private final SendableChooser<Command> autoChooser;

    public RobotContainer() {
        // Named commands must be registered before building the auto chooser
        Autos.registerNamedCommands(fuelSubsystem, climberSubsystem);

        autoChooser = AutoBuilder.buildAutoChooser("ShootOnly");
        SmartDashboard.putData("Auto Mode", autoChooser);

        configureBindings();

        CommandScheduler.getInstance().schedule(FollowPathCommand.warmupCommand());
    }

    private void configureBindings() {
        drivetrain.setDefaultCommand(
            drivetrain.applyRequest(() ->
                drive.withVelocityX(-driverController.getLeftY() * MaxSpeed * turtlemode)
                    .withVelocityY(-driverController.getLeftX() * MaxSpeed * turtlemode)
                    .withRotationalRate(-driverController.getRightX() * MaxAngularRate)
            )
        );

        final var idle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled().whileTrue(
            drivetrain.applyRequest(() -> idle).ignoringDisable(true)
        );

        driverController.a().whileTrue(drivetrain.applyRequest(() -> brake));
        driverController.b().whileTrue(drivetrain.applyRequest(() ->
            point.withModuleDirection(new Rotation2d(-driverController.getLeftY(), -driverController.getLeftX()))
        ));

        driverController.povUp().whileTrue(drivetrain.applyRequest(() ->
            forwardStraight.withVelocityX(0.5).withVelocityY(0))
        );
        driverController.povDown().whileTrue(drivetrain.applyRequest(() ->
            forwardStraight.withVelocityX(-0.5).withVelocityY(0))
        );

        driverController.back().and(driverController.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        driverController.back().and(driverController.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        driverController.start().and(driverController.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        driverController.start().and(driverController.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        driverController.leftBumper().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));
        driverController.rightBumper().whileTrue(Commands.startEnd(() -> turtlemode = .2, () -> turtlemode = 1.0));

        // Intake: spin up rollers while held, stop on release
        operatorController.leftBumper().whileTrue(
            fuelSubsystem.startEnd(
                () -> {
                    fuelSubsystem.setIntakeLauncherRoller(-.8 * SmartDashboard.getNumber("Intaking intake roller value", INTAKE_INTAKING_PERCENT));
                    fuelSubsystem.setFeederRoller(-.8 * SmartDashboard.getNumber("Intaking feeder roller value", INDEXER_INTAKING_PERCENT));
                },
                fuelSubsystem::stop
            )
        );

        // Launch sequence: spin up for SPIN_UP_SECONDS then feed, stop on release
        operatorController.rightBumper().whileTrue(
            Commands.sequence(
                fuelSubsystem.startEnd(
                    () -> {
                        fuelSubsystem.setIntakeLauncherRoller(-.8 * SmartDashboard.getNumber("Launching launcher roller value", LAUNCHING_LAUNCHER_PERCENT));
                        fuelSubsystem.setFeederRoller(.8 * SmartDashboard.getNumber("Launching spin-up feeder value", INDEXER_SPIN_UP_PRE_LAUNCH_PERCENT));
                    },
                    () -> {}
                ).withTimeout(SPIN_UP_SECONDS),
                fuelSubsystem.startEnd(
                    () -> {
                        fuelSubsystem.setIntakeLauncherRoller(-.8 * SmartDashboard.getNumber("Launching launcher roller value", LAUNCHING_LAUNCHER_PERCENT));
                        fuelSubsystem.setFeederRoller(.8 * SmartDashboard.getNumber("Launching feeder roller value", INDEXER_LAUNCHING_PERCENT));
                    },
                    fuelSubsystem::stop
                )
            )
        );

        // Eject: reverse rollers while held, stop on release
        operatorController.a().whileTrue(
            fuelSubsystem.startEnd(
                () -> {
                    fuelSubsystem.setIntakeLauncherRoller(SmartDashboard.getNumber("Intaking intake roller value", INTAKE_EJECT_PERCENT));
                    fuelSubsystem.setFeederRoller(-1 * SmartDashboard.getNumber("Intaking intake roller value", INDEXER_LAUNCHING_PERCENT));
                },
                fuelSubsystem::stop
            )
        );

        // Climber
        operatorController.povDown().whileTrue(
            climberSubsystem.startEnd(
                () -> climberSubsystem.setClimber(CLIMBER_MOTOR_DOWN_PERCENT),
                climberSubsystem::stop
            )
        );
        operatorController.povUp().whileTrue(
            climberSubsystem.startEnd(
                () -> climberSubsystem.setClimber(CLIMBER_MOTOR_UP_PERCENT),
                climberSubsystem::stop
            )
        );

        drivetrain.registerTelemetry(logger::telemeterize);

        fuelSubsystem.setDefaultCommand(fuelSubsystem.run(fuelSubsystem::stop));
        climberSubsystem.setDefaultCommand(climberSubsystem.run(climberSubsystem::stop));
    }

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }
}
