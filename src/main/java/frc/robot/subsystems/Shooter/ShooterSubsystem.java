package frc.robot.subsystems.Shooter;

import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterSubsystem extends SubsystemBase {

    private final SparkFlex shooterMotor;
    private final SparkMax intakeMotor;
    private final SparkMax indexerMotor; 
    private SparkClosedLoopController shooter_pidController;
    private static final int shooterID = 12;
    private static final int intakeID = 11;
    private static final int indexID = 10;
    private SparkMax shooterAngleMotor = new SparkMax( 9, MotorType.kBrushless);
    
    public ShooterSubsystem() {
        shooterMotor = new SparkFlex(shooterID, MotorType.kBrushless);
        intakeMotor = new SparkMax(intakeID, MotorType.kBrushless);
        indexerMotor = new SparkMax(indexID, MotorType.kBrushless);
        shooter_pidController = shooterMotor.getClosedLoopController();
        SparkFlexConfig shooterConfig = new SparkFlexConfig();
        SparkMaxConfig intakeConfig = new SparkMaxConfig();
        SparkMaxConfig indexerConfig = new SparkMaxConfig();
        SparkMaxConfig angleConfig = new SparkMaxConfig();
        shooterConfig.closedLoop.pid(0.0001, 0.0, 0.0);
        shooterConfig.closedLoop.velocityFF(0.000155);
        
        intakeConfig.inverted(true);
        indexerConfig.inverted(true);
        angleConfig.closedLoop.pid(0.15, 0.0, 0.05);
        angleConfig.softLimit.forwardSoftLimit(0)
                             .forwardSoftLimitEnabled(true)
                             .reverseSoftLimit(-95.2)
                             .reverseSoftLimitEnabled(true);
        shooterMotor.configure(shooterConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        intakeMotor.configure(intakeConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        indexerMotor.configure(indexerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        shooterAngleMotor.configure(angleConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    public void setShooterAngleSpeed(double speed) {
        shooterAngleMotor.set(speed);
    }

    public void shootLongRange(){
        double targetRPM = 3400;
        shooter_pidController.setSetpoint(targetRPM, ControlType.kVelocity);
        intakeMotor.set(0.83);
        indexerMotor.set(0.1);
    }
    public void shootShortRange(){
        double targetRPM = 2900;
        shooter_pidController.setSetpoint(targetRPM, ControlType.kVelocity);
        intakeMotor.set(0.83);
        indexerMotor.set(1);
    }

    public void shooterON(){
         double targetRPM = 3400;
        shooter_pidController.setSetpoint(targetRPM, ControlType.kVelocity); 
    }

    public void shooterOFF(){
         double targetRPM = 0;
        shooter_pidController.setSetpoint(targetRPM, ControlType.kVelocity); 
    }

    public void indexerON(){
        intakeMotor.set(0.83);
        indexerMotor.set(0.1);
    }

    public void indexerOFF(){
        intakeMotor.set(0.0);
        indexerMotor.set(0.0);
    }

    public void setPosition(double position){
        shooterAngleMotor.getClosedLoopController().setSetpoint(position, ControlType.kPosition);
    }
    
    public void OuttakeIndex(){
        indexerMotor.set(-0.25);
    }

    public void stop(){
        intakeMotor.set(0);
        shooterMotor.set(0);
        indexerMotor.set(0);
        intakeMotor.disable();
        shooterMotor.disable();
    indexerMotor.disable();
    
    }
    
}