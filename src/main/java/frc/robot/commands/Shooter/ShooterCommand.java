package frc.robot.commands.Shooter;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.Lights.Lights;
import frc.robot.subsystems.Shooter.ShooterSubsystem;

public class ShooterCommand extends Command {
    private final ShooterSubsystem shooterSubsystem;
    @SuppressWarnings("unused") // Fixes the warning given by the lights variable being unused
    private final Lights lights;
    private boolean IsLimelightOn = true;
    private static final int[] redTagsToUse = { 8, 10, 12 };
    private static final int[] blueTagsToUse = { 24, 26, 27 };

    public ShooterCommand(ShooterSubsystem shooterSubsystem, Lights lights) {
        this.shooterSubsystem = shooterSubsystem;
        this.lights = lights;
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
        double rightTriggerAxis = Constants.OperatorConstants.operatorXbox.getRightTriggerAxis();
        double leftTriggerAxis  = Constants.OperatorConstants.operatorXbox.getLeftTriggerAxis();
        double rightXAxis = Constants.OperatorConstants.operatorXbox.getRightX();
        Trigger Bbutton = Constants.OperatorConstants.operatorXbox.b();
        Trigger Xbutton    = Constants.OperatorConstants.operatorXbox.x();
        Trigger Lbumper    = Constants.OperatorConstants.operatorXbox.leftBumper();
        Trigger Dpad_left  = Constants.OperatorConstants.operatorXbox.povLeft();
        Trigger Dpad_up    = Constants.OperatorConstants.operatorXbox.povUp();
        Trigger Dpad_right = Constants.OperatorConstants.operatorXbox.povRight();
        Trigger Dpad_down  = Constants.OperatorConstants.operatorXbox.povDown();

        if (Dpad_left.getAsBoolean()) {
            shooterSubsystem.setPosition(-16);
        } else if (Dpad_up.getAsBoolean()) {
            shooterSubsystem.setPosition(-6.1);
        } else if (Dpad_right.getAsBoolean()) {
            shooterSubsystem.setPosition(0);
        } else if (Dpad_down.getAsBoolean()) {
            shooterSubsystem.setShooterAngleSpeed((rightXAxis * rightXAxis * rightXAxis) / 5.0);
        } else if (Bbutton.getAsBoolean()) {
            if (IsLimelightOn && LimelightHelpers.getTA("limelight") > 0) {
                double tx = LimelightHelpers.getTX("limelight");
                shooterSubsystem.setPosition(shooterSubsystem.getPosition() + tx/9.0);
            }
        }
        
        if (Lbumper.getAsBoolean()) {
            shooterSubsystem.shooterON(6000);
            if (rightTriggerAxis > 0.2) {
                shooterSubsystem.indexerON();
            } else {
                shooterSubsystem.indexerOFF();
            }
        } else if (leftTriggerAxis > 0.2) {
            shooterSubsystem.shooterON(3150);
            if (rightTriggerAxis > 0.2) {
                shooterSubsystem.indexerON();
            } else {
                shooterSubsystem.indexerOFF();
            }
        } else if (rightTriggerAxis > 0.2) {
            shooterSubsystem.indexerON();
        } else if (Xbutton.getAsBoolean()) {
            shooterSubsystem.reverseIndexer();
        } else {
            shooterSubsystem.stop();
        }
    }

    @Override
    public void end(boolean interrupted) {
        shooterSubsystem.stop();
    }
}
