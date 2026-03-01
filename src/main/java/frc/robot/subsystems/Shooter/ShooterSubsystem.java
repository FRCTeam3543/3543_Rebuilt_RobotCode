package frc.robot.subsystems.Shooter;

import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;

public class ShooterSubsystem extends SubsystemBase {

    private final SparkFlex shooterMotor;
    private final SparkMax intakeMotor;
    private final SparkMax indexerMotor; 
    private SparkClosedLoopController shooter_pidController;
    private static final int shooterID = 12;
    private static final int intakeID = 11;
    private static final int indexID = 10;
    private SparkMax shooterAngleMotor = new SparkMax( 9, MotorType.kBrushless);
    
    @Override
    public void periodic() {
        // double tx = LimelightHelpers.getTX("limelight");
        // shooterAngleMotor.set(tx * 0.055);

         //NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
         //double TagId = table.getEntry("tid").getDouble(0);
         //System.out.println(TagId);
         //boolean LockedOn;
         //if (TagId == 1) {
            //LockedOn = true;
         //}
             //shooterAngleMotor.set(tx * 0.055);

        //if (Math.abs(tx) >= 16) {
            //shooterAngleMotor.set(tx * 0.055);
        //} else if (tx >= 0) {
            //shooterAngleMotor.set(Math.sqrt(tx));
        //} else {
            //shooterAngleMotor.set(Math.sqrt(-tx));
        //}
       
        //System.out.println(tx);
    }
    
    public ShooterSubsystem() {
        shooterMotor = new SparkFlex(shooterID, MotorType.kBrushless);
        intakeMotor = new SparkMax(intakeID, MotorType.kBrushless);
        indexerMotor = new SparkMax(indexID, MotorType.kBrushless);
        shooter_pidController = shooterMotor.getClosedLoopController();
        SparkFlexConfig shooterConfig = new SparkFlexConfig();
        SparkMaxConfig intakeConfig = new SparkMaxConfig();
        SparkMaxConfig indexerConfig = new SparkMaxConfig();
        shooterConfig.closedLoop.pid(0.0001, 0.0, 0.0);
        shooterConfig.closedLoop.velocityFF(0.00017);
        intakeConfig.inverted(true);
        indexerConfig.inverted(true);
        shooterMotor.configure(shooterConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        intakeMotor.configure(intakeConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        indexerMotor.configure(indexerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    public void setShooterAngleSpeed(double speed) {
        shooterAngleMotor.set(speed);
    }

    public void shoot(){
        double targetRPM = 3700;
        shooter_pidController.setSetpoint(targetRPM, ControlType.kVelocity);
        intakeMotor.set(0.83);
        indexerMotor.set(0.1);
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