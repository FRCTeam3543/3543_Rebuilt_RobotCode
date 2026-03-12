package frc.robot.commands.Shooter;

import edu.wpi.first.math.controller.PIDController;
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
    private final PIDController shooterAnglePID = new PIDController(0.025, 0, 0);
    private final Lights lights;
    private boolean IsLimelightOn = true;
    private static final int[] redTagsToUse = { 8, 10, 2 };
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
                lights.setColor(Constants.LightsConstants.Colors.BLUE);
            } else {
                // tell the limelight to ignore all tags except red hub tags when we're on red
                LimelightHelpers.SetFiducialIDFiltersOverride("limelight", redTagsToUse);
                lights.setColor(Constants.LightsConstants.Colors.RED);
            }
        }
    }

    @Override
    public void execute() {
        double rightTriggerAxis = Constants.OperatorConstants.operatorXbox.getRightTriggerAxis();
        // Trigger Xbutton = Constants.OperatorConstants.operatorXbox.button(3);
        // Trigger Ybutton = Constants.OperatorConstants.operatorXbox.button(4);
        // Trigger Bbutton = Constants.OperatorConstants.operatorXbox.button(2);
        Trigger Lbumper = Constants.OperatorConstants.operatorXbox.button(5);
        Trigger Rbumper = Constants.OperatorConstants.operatorXbox.button(6);
        Trigger Dpad_left = Constants.OperatorConstants.operatorXbox.pov(270);
        Trigger Dpad_up = Constants.OperatorConstants.operatorXbox.pov(0);
        Trigger Dpad_right = Constants.OperatorConstants.operatorXbox.pov(90);
        
        if (rightTriggerAxis > 0.2) {
            shooterSubsystem.shootLongRange();
        } else if (Rbumper.getAsBoolean()) {
            shooterSubsystem.shootShortRange();
        } else {
            shooterSubsystem.stop();
        }
        if (Lbumper.getAsBoolean()){
            shooterSubsystem.OuttakeIndex();
        }

        if (Dpad_left.getAsBoolean()) {
            shooterSubsystem.setPosition(-95.2);
            System.out.println("X pressed");
        } else if (Dpad_up.getAsBoolean()) {
            shooterSubsystem.setPosition(-46.0);
            System.out.println("Y pressed");
        } else if (Dpad_right.getAsBoolean()) {
            shooterSubsystem.setPosition(0.0);
            System.out.println("B pressed");
        } else {
            if (LimelightHelpers.getTA("limelight") > 0) {
                lights.setColor(Constants.LightsConstants.Colors.GREEN);

                double tx = LimelightHelpers.getTX("limelight");
                double speed = shooterAnglePID.calculate(-tx, 0);
                shooterSubsystem.setShooterAngleSpeed(speed);
            } else {
                var alliance = DriverStation.getAlliance();
                if (alliance.isPresent()) {
                    if (alliance.get() == Alliance.Blue) {
                        lights.setColor(Constants.LightsConstants.Colors.BLUE);
                    } else {
                        lights.setColor(Constants.LightsConstants.Colors.RED);
                    }
                }
            }
        }
    }

    @Override
    public void end(boolean interrupted) {
        shooterSubsystem.stop();
    }
}
