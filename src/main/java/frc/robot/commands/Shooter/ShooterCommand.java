package frc.robot.commands.Shooter;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.Shooter.ShooterSubsystem;

public class ShooterCommand extends Command {
    private final ShooterSubsystem shooterSubsystem;
    private final PIDController shooterAnglePID = new PIDController(0.055, 0, 0);

    private static final int[] redTagsToUse = { 8, 10, 11 };
    private static final int[] blueTagsToUse = { 24, 26, 27 };

    public ShooterCommand(ShooterSubsystem shooterSubsystem) {
        this.shooterSubsystem = shooterSubsystem;
        addRequirements(shooterSubsystem);
    }

    // runs when the command starts
    @Override
    public void initialize() {
        // check what alliance we're on
        var alliance = DriverStation.getAlliance();
        if (alliance.isPresent()) {
            if (alliance.get() == Alliance.Blue) {
                // tell the limelight to ignore all tags except blue hub tags when we're on blue
                LimelightHelpers.SetFiducialIDFiltersOverride("limelight", blueTagsToUse);
            } else {
                // tell the limelight to ignore all tags except red hub tags when we're on red
                LimelightHelpers.SetFiducialIDFiltersOverride("limelight", redTagsToUse);
            }
        }
    }

    @Override
    public void execute() {
        double rightTriggerAxis = Constants.OperatorConstants.driverXbox.getRightTriggerAxis();

        if (rightTriggerAxis > 0.2) {
            shooterSubsystem.shoot();
        }
        else{
            shooterSubsystem.stop();
        }

        double tx = LimelightHelpers.getTX("limelight");
        // setpoint of 0 tells the PID controller to move the motor until tx is 0 (so the shooter is facing the tag)
        double speed = shooterAnglePID.calculate(-tx, 0);
        // SmartDashboard.putNumber("Shooter Angle Motor Speed", speed);
        shooterSubsystem.setShooterAngleSpeed(speed);
    }

    @Override
    public void end(boolean interrupted) {
        shooterSubsystem.stop();
    }
}
