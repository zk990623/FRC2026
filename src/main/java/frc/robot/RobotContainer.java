// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import com.pathplanner.lib.commands.FollowPathCommand;
import com.pathplanner.lib.commands.PathPlannerAuto;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.subsystems.Drivetrain.CommandSwerveDrivetrain;
import frc.robot.subsystems.Drivetrain.TunerConstants;

public class RobotContainer {

    private double MaxSpeed = (4 / 5.47) * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
    private double MaxTeleOpSpeed = MaxSpeed;
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond);

    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxTeleOpSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1)
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);
    private final SwerveRequest.RobotCentric forwardStraight = new SwerveRequest.RobotCentric()
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final CommandXboxController joystick = new CommandXboxController(0);

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

    public RobotContainer() {

        configureBindings();

        CommandScheduler.getInstance().schedule(FollowPathCommand.warmupCommand());
    }
    private void configureBindings() {

        drivetrain.setDefaultCommand(
                drivetrain.applyRequest(() -> drive
                        .withVelocityX(-joystick.getLeftY() * MaxSpeed)
                        .withVelocityY(-joystick.getLeftX() * MaxSpeed)
                        .withRotationalRate(-joystick.getRightX() * MaxAngularRate)));
        final var idle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled().whileTrue(
                drivetrain.applyRequest(() -> idle).ignoringDisable(true));
        drivetrain.registerTelemetry(logger::telemeterize);

        // joystick.start().onTrue(drivetrain.runOnce(drivetrain::resetPosetotest));

        // joystick.rightBumper().whileTrue(this.superstructure.DriveToTrench());

        // joystick.leftBumper().whileTrue(this.superstructure.shoot());

        // joystick.a().onTrue(
        //         Commands.runOnce(() -> this.shooter.hoodUp(), this.shooter));

        // joystick.b().onTrue(
        //         Commands.runOnce(() -> this.shooter.hoodDown(), this.shooter));

        // joystick.leftBumper().onTrue(
        //         Commands.runOnce(() -> this.shooter.flywheelup(), this.shooter));

        // joystick.rightBumper().onTrue(
        //         Commands.runOnce(() -> this.shooter.flywheeldown(), this.shooter));
        // // sysidTest();
    }

    public PathPlannerAuto getAutonomousCommand() {
        return new PathPlannerAuto("new Auto");
    }

    public void sysidTest() {
        joystick.povUp().whileTrue(drivetrain.applyRequest(() -> forwardStraight.withVelocityX(0.5).withVelocityY(0)));
        joystick.povDown()
                .whileTrue(drivetrain.applyRequest(() -> forwardStraight.withVelocityX(-0.5).withVelocityY(0)));

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        joystick.back().and(joystick.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        joystick.back().and(joystick.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        joystick.start().and(joystick.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        joystick.start().and(joystick.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

    }
}