// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Intake;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;
import frc.robot.commands.Shooter.ShooterCommand;
import frc.robot.subsystems.Intake.IntakeSubsystem;
import frc.robot.subsystems.Shooter.ShooterSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class IntakeCommand extends Command {
  private final IntakeSubsystem intakeSubsystem;
  public IntakeCommand(IntakeSubsystem intakeSubsystem) {
  this.intakeSubsystem = intakeSubsystem;
  addRequirements (intakeSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
      double leftTriggerAxis = Constants.OperatorConstants.driverXbox.getLeftTriggerAxis();
      Trigger RightBumper = Constants.OperatorConstants.operatorXbox.button(6);
        if (leftTriggerAxis > 0.2) {
            intakeSubsystem.setIntakeRoller(-1);
            intakeSubsystem.setIntakePosition(5.7);
        }
        else{
            intakeSubsystem.setIntakeRoller(0);
            intakeSubsystem.setIntakePosition(.6);
        }
        if (RightBumper.getAsBoolean()) {
          intakeSubsystem.setIntakeRoller(1);
          
        }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intakeSubsystem.setIntakeRoller(0);
    intakeSubsystem.setIntakePosition(.6);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
