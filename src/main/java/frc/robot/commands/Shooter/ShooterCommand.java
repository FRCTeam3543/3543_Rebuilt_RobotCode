package frc.robot.commands.Shooter;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
    private boolean toggledLimelight = false;
    private static final int[] redTagsToUse = { 8, 10, 12 };
    private static final int[] blueTagsToUse = { 24, 26, 27 };
    SendableChooser<String> operatorControlsChooser;

    public ShooterCommand(ShooterSubsystem shooterSubsystem, Lights lights) {
        operatorControlsChooser = new SendableChooser<String>();
        operatorControlsChooser.setDefaultOption("Autumn", "autumn");
        operatorControlsChooser.addOption("Testing", "testing");
        operatorControlsChooser.addOption("Isaiah", "isaiah");
        SmartDashboard.putData("Operator Controls Chooser", operatorControlsChooser);
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
        Trigger reverseButton = Constants.OperatorConstants.driverXbox.x();
        Trigger Abutton    = Constants.OperatorConstants.operatorXbox.button(1);
        Trigger Bbutton    = Constants.OperatorConstants.operatorXbox.button(2);
        Trigger Xbutton    = Constants.OperatorConstants.operatorXbox.button(3);
        Trigger Lbumper    = Constants.OperatorConstants.operatorXbox.button(5);
        Trigger Rbumper    = Constants.OperatorConstants.operatorXbox.button(6);
        Trigger Dpad_left  = Constants.OperatorConstants.operatorXbox.pov(  270);
        Trigger Dpad_up    = Constants.OperatorConstants.operatorXbox.pov(    0);
        Trigger Dpad_right = Constants.OperatorConstants.operatorXbox.pov(   90);
        Trigger Dpad_down  = Constants.OperatorConstants.operatorXbox.pov(  180);

        if (Dpad_left.getAsBoolean()) {
            shooterSubsystem.setPosition(-16);
        } else if (Dpad_up.getAsBoolean()) {
            shooterSubsystem.setPosition(-6.1);
        } else if (Dpad_right.getAsBoolean()) {
            shooterSubsystem.setPosition(0);
        } else if (Dpad_down.getAsBoolean()) {} else {
            if (IsLimelightOn && LimelightHelpers.getTA("limelight") > 0) {
                double tx = LimelightHelpers.getTX("limelight");
                shooterSubsystem.setShooterAngleSpeed(tx / 90);
            }
        }

        if (operatorControlsChooser.getSelected() == "autumn") {
            // Autumn's preferred control scheme
            if (Bbutton.getAsBoolean()) {
                shooterSubsystem.reverseIndexer();
            } else if (Lbumper.getAsBoolean()) {
                shooterSubsystem.shooterON(6000);
                if (leftTriggerAxis > 0.2) {
                    shooterSubsystem.indexerON();
                } else {
                    shooterSubsystem.indexerOFF();
                }
            } else if (leftTriggerAxis > 0.2) {
                shooterSubsystem.indexerON();
                shooterSubsystem.spinup();
            } else if (rightTriggerAxis > 0.2) {
                shooterSubsystem.shootLongRange();
            } else if (Rbumper.getAsBoolean()) {
                shooterSubsystem.shootShortRange();
            } else if (Xbutton.getAsBoolean() && !toggledLimelight) {
                IsLimelightOn = !IsLimelightOn;
                toggledLimelight = true;
            } else {
                shooterSubsystem.stop();
                toggledLimelight = false;
            }

        } else if (operatorControlsChooser.getSelected() == "isaiah") {
            // Isaiah's preferred control scheme
            if (leftTriggerAxis > 0.2) {
                shooterSubsystem.shooterON(6000);
            } else if (Abutton.getAsBoolean()) {
                shooterSubsystem.reverseIndexer();
            } else if (rightTriggerAxis > 0.2) {
                shooterSubsystem.shooterON(3000);
            } else if (Rbumper.getAsBoolean()) {
                shooterSubsystem.shooterON(3000);
                shooterSubsystem.indexerON();
            } else if (Lbumper.getAsBoolean()) {
                shooterSubsystem.shooterON(6000);
                shooterSubsystem.indexerON();
            } else if (Xbutton.getAsBoolean()) {
                IsLimelightOn = !IsLimelightOn;
            } else {
                shooterSubsystem.stop();
            }
            
        } else {
            // Testing Controls
            if (rightTriggerAxis > 0.2) {
                shooterSubsystem.spinup();
                if (Xbutton.getAsBoolean()) {
                    shooterSubsystem.indexerON();
                } else {
                    shooterSubsystem.indexerOFF();
                }
            } else {
                shooterSubsystem.stop();
            }
        }

        if (reverseButton.getAsBoolean()) {
            shooterSubsystem.reverseIndexer();
        }
    }

    @Override
    public void end(boolean interrupted) {
        shooterSubsystem.stop();
    }
}
