package frc.robot.commands.Shooter;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Shooter.ShooterSubsystem;

public class ShooterCommand extends Command {
    private final ShooterSubsystem shooterSubsystem;
    public ShooterCommand(ShooterSubsystem shooterSubsystem) {
        this.shooterSubsystem = shooterSubsystem;
        addRequirements(shooterSubsystem);
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
    }

    @Override
    public void end(boolean interrupted) {
        shooterSubsystem.stop();
    }
}
