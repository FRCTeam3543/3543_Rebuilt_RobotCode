// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Intake;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase {
  /** Creates a new IntakeSubsystem. */
  private final SparkFlex intakeFrontRight = new SparkFlex(15, MotorType.kBrushless);
  private final SparkMax intakeRoller = new SparkMax(13, MotorType.kBrushless);

  public IntakeSubsystem() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
public void setIntakePosition (double position) {
  intakeFrontRight.getClosedLoopController().setSetpoint(position, ControlType.kPosition);
}
public void setIntakeRoller (double speed) {
  intakeRoller.set(speed);
}
}
