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
import com.pathplanner.lib.auto.NamedCommands;
import swervelib.SwerveInputStream;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.commands.Shooter.ShooterCommand;
import frc.robot.commands.Intake.IntakeCommand;

public class RobotContainer {

        private final SwerveSubsystem drivebase = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(),
                        "swerve/neo"));

        private final Lights lights = new Lights();
        private final ShooterSubsystem shooterSubsystem = new ShooterSubsystem();
        private final IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
        private final SendableChooser<String> autoChooser = new SendableChooser<>();

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
                         .withControllerRotationAxis(() -> Constants.OperatorConstants.driverXbox.getRawAxis(
                                         2))
                         .deadband(OperatorConstants.DEADBAND)
                         .scaleTranslation(0.8)
                         .allianceRelativeControl(true);

         SwerveInputStream driveDirectAngleKeyboard = driveAngularVelocityKeyboard.copy()
                         .withControllerHeadingAxis(() -> Math.sin(
                                         Constants.OperatorConstants.driverXbox.getRawAxis(
                                                         2) *
                                                         Math.PI)
                                        *
                                        (Math.PI *
                                                         2),
                                         () -> Math.cos(
                                                         Constants.OperatorConstants.driverXbox.getRawAxis(
                                                                         2) *
                                                                         Math.PI)
                                                         *
                                                         (Math.PI *
                                                                         2))
                         .headingWhile(true);

        public RobotContainer() {

                configureBindings();
                DriverStation.silenceJoystickConnectionWarning(true);

                shooterSubsystem.setDefaultCommand(new ShooterCommand(shooterSubsystem));
                intakeSubsystem.setDefaultCommand(new IntakeCommand(intakeSubsystem));
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
                /*  new Trigger(() ->
                 Constants.OperatorConstants.driverXbox.button(5).getAsBoolean() ||
                  Constants.OperatorConstants.driverXbox.button(6).getAsBoolean() ||
                  Constants.OperatorConstants.driverXbox.button(7).getAsBoolean())
                  .onTrue(new EuroVision(drivebase, null, null, null, null, m_limelight)); */

                 if (RobotBase.isSimulation()) {
                         drivebase.setDefaultCommand(driveFieldOrientedDirectAngleKeyboard);
                 } else {
                         drivebase.setDefaultCommand(driveFieldOrientedAnglularVelocity);
                 }

         }

         //public Command getAutonomousCommand() {
         //String selectedAuto = autoChooser.getSelected();
              // return Commands.print("hello");
             //return drivebase.getAutonomousCommand(selectedAuto);
        // }

         public void setMotorBrake(boolean brake) {
                 drivebase.setMotorBrake(brake);
         }
 }