// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Intake;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;
import frc.robot.subsystems.Intake.IntakeSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class IntakeCommand extends Command {
  private final IntakeSubsystem intakeSubsystem;
  private boolean isLocked = false;

  public IntakeCommand(IntakeSubsystem intakeSubsystem) {
    this.intakeSubsystem = intakeSubsystem;
    addRequirements(intakeSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double leftTrigger = Constants.OperatorConstants.driverXbox.getLeftTriggerAxis();
    Trigger YButton = Constants.OperatorConstants.operatorXbox.y();

    if (leftTrigger > 0.2) {
      intakeSubsystem.setIntakePosition(6.0);
      if (intakeSubsystem.getIntakePosition() >= 5.2) {
        intakeSubsystem.setIntakeRoller(-1);
      }
    }
    else{
      intakeSubsystem.setIntakeRoller(0);
      intakeSubsystem.setIntakePosition(2);
    }
    if (YButton.getAsBoolean()) {
      isLocked = true;
    }
    if (isLocked) {
      intakeSubsystem.setIntakePosition(0);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intakeSubsystem.setIntakeRoller(0);
    intakeSubsystem.setIntakePosition(2);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
