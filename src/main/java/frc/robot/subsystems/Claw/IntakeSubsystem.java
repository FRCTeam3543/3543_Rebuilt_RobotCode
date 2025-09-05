package frc.robot.subsystems.Claw;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.Lights.Lights;

public class IntakeSubsystem extends SubsystemBase {

    private final SparkMax intakeMotor;
    private final DigitalInput intakeLimitSwitch;
    private final Lights lights;  // Lights instance

    public IntakeSubsystem(Lights lights) {  // Pass Lights as a parameter
        this.lights = lights; // Initialize Lights properly
        intakeMotor = new SparkMax(9, MotorType.kBrushless);
        intakeLimitSwitch = new DigitalInput(1);

        SparkMaxConfig config = new SparkMaxConfig();
        config.smartCurrentLimit(40)
              .openLoopRampRate(0.5)
              .idleMode(IdleMode.kBrake)
              .inverted(false)
              .closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder)
              .maxMotion.maxVelocity(2000)
              .maxAcceleration(1000)
              .allowedClosedLoopError(1.0);

        intakeMotor.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
    }

    public boolean isHoldingCoral() {
        return intakeLimitSwitch.get();
    }

    @Override
    public void periodic() {
        if (isHoldingCoral()) {
            lights.setColor(Constants.LightsConstants.Colors.RED); // Green when holding
        } else {
            lights.setColor(Constants.LightsConstants.Colors.YELLOW); // Yellow when not holding
        }
    }

    public void launch() {
        this.runIntake(-1.0);
    }

    public void intake() {
        this.runIntake(0.7);
    }

    public void runIntake(double speed) {
        intakeMotor.set(speed);
    }

    public void stop() {
        intakeMotor.set(0);
        intakeMotor.disable();
    }
}