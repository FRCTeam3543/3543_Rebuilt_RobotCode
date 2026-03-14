// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.Intake.IntakeSubsystem;
import frc.robot.subsystems.Lights.Lights;
import frc.robot.subsystems.Shooter.ShooterSubsystem;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import java.io.File;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import swervelib.SwerveInputStream;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.commands.Shooter.ShooterCommand;

public class RobotContainer {

        private final SwerveSubsystem drivebase = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(),
                        "swerve/neo"));

        public final ShooterSubsystem shooterSubsystem = new ShooterSubsystem();
        public final IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
        public final Lights lights = new Lights();
        private final ShooterCommand shooterCommand = new ShooterCommand(shooterSubsystem, lights);
        private final SendableChooser<Command> autoChooser;

        SwerveInputStream driveAngularVelocity = SwerveInputStream.of(drivebase.getSwerveDrive(),
                () -> Constants.OperatorConstants.driverXbox.getLeftY() * -1,
                () -> Constants.OperatorConstants.driverXbox.getLeftX() * -1)
                .withControllerRotationAxis(() -> Constants.OperatorConstants.driverXbox.getRightX() * -1)
                .deadband(OperatorConstants.DEADBAND)
                .scaleTranslation(0.8)
                .allianceRelativeControl(false);

        SwerveInputStream driveDirectAngle = driveAngularVelocity.copy()
                .withControllerHeadingAxis(Constants.OperatorConstants.driverXbox::getRightX,
                        Constants.OperatorConstants.driverXbox::getRightY)
                .headingWhile(true);

        SwerveInputStream driveRobotOriented = driveAngularVelocity.copy().robotRelative(true)
                .allianceRelativeControl(false);

        SwerveInputStream driveAngularVelocityKeyboard = SwerveInputStream.of(drivebase.getSwerveDrive(),
                        () -> -Constants.OperatorConstants.driverXbox.getLeftY() * 1,
                        () -> -Constants.OperatorConstants.driverXbox.getLeftX() * 1)
                .withControllerRotationAxis(() -> Constants.OperatorConstants.driverXbox.getRawAxis(2))
                .deadband(OperatorConstants.DEADBAND)
                .scaleTranslation(0.8)
                .allianceRelativeControl(true);

        SwerveInputStream driveDirectAngleKeyboard = driveAngularVelocityKeyboard.copy()
                .withControllerHeadingAxis(() -> Math.sin(Constants.OperatorConstants.driverXbox.getRawAxis(2)
                                                * Math.PI) * (Math.PI * 2), 
                                        () -> Math.cos(Constants.OperatorConstants.driverXbox.getRawAxis(2) 
                                                * Math.PI) * (Math.PI * 2))
                .headingWhile(true);

        public RobotContainer() {
                NamedCommands.registerCommand("setIntakeOn",
                                Commands.runOnce(() -> intakeSubsystem.setIntakeRoller(0.5), intakeSubsystem));
                NamedCommands.registerCommand("setIntakeOff",
                                Commands.runOnce(() -> intakeSubsystem.setIntakeRoller(0), intakeSubsystem));
                NamedCommands.registerCommand("rotateTurretFullLeft",
                                Commands.runOnce(() -> shooterSubsystem.setPosition(-95.2), shooterSubsystem));
                NamedCommands.registerCommand("zeroTurret",
                                Commands.runOnce(() -> shooterSubsystem.setPosition(0), shooterSubsystem));
                NamedCommands.registerCommand("shooterON",
                                Commands.runOnce(() -> shooterSubsystem.shooterON(), shooterSubsystem));
                NamedCommands.registerCommand("shooterOFF",
                                Commands.runOnce(() -> shooterSubsystem.shooterOFF(), shooterSubsystem));
                NamedCommands.registerCommand("indexerON",
                                Commands.runOnce(() -> shooterSubsystem.indexerON(), shooterSubsystem));
                NamedCommands.registerCommand("indexerOFF",
                                Commands.runOnce(() -> shooterSubsystem.indexerOFF(), shooterSubsystem));
                NamedCommands.registerCommand("intakeDown",
                                Commands.runOnce(() -> intakeSubsystem.setIntakePosition(3.5), intakeSubsystem));
                NamedCommands.registerCommand("intakeMid",
                                Commands.runOnce(() -> intakeSubsystem.setIntakePosition(2.9), intakeSubsystem));
                NamedCommands.registerCommand("intakeUp",
                                Commands.runOnce(() -> intakeSubsystem.setIntakePosition(0), intakeSubsystem));

                autoChooser = AutoBuilder.buildAutoChooser("Auto Left");
                SmartDashboard.putData("Auto Chooser", autoChooser);
                configureBindings();
                DriverStation.silenceJoystickConnectionWarning(true);

                shooterSubsystem.setDefaultCommand(shooterCommand);

                
        }

        private void configureBindings() {
                Command driveFieldOrientedDirectAngle = drivebase.driveFieldOriented(driveDirectAngle);
                Command driveFieldOrientedAnglularVelocity = drivebase.driveFieldOriented(driveAngularVelocity);
                Command driveRobotOrientedAngularVelocity = drivebase.driveFieldOriented(driveRobotOriented);
                Command driveSetpointGen = drivebase.driveWithSetpointGeneratorFieldRelative(
                                driveDirectAngle);
                Command driveFieldOrientedDirectAngleKeyboard = drivebase.driveFieldOriented(driveDirectAngleKeyboard);
                Command driveFieldOrientedAnglularVelocityKeyboard = drivebase
                                .driveFieldOriented(driveAngularVelocityKeyboard);
                Command driveSetpointGenKeyboard = drivebase.driveWithSetpointGeneratorFieldRelative(
                                driveDirectAngleKeyboard);
                /*
                 * new Trigger(() ->
                 * Constants.OperatorConstants.driverXbox.button(5).getAsBoolean() ||
                 * Constants.OperatorConstants.driverXbox.button(6).getAsBoolean() ||
                 * Constants.OperatorConstants.driverXbox.button(7).getAsBoolean())
                 * .onTrue(new EuroVision(drivebase, null, null, null, null, m_limelight));
                 */

                if (RobotBase.isSimulation()) {
                        drivebase.setDefaultCommand(driveFieldOrientedDirectAngleKeyboard);
                } else {
                        drivebase.setDefaultCommand(driveFieldOrientedAnglularVelocity);
                }

        }

        public Command getAutonomousCommand() {
                return autoChooser.getSelected();
        }

        public void setMotorBrake(boolean brake) {
                drivebase.setMotorBrake(brake);
        }
}