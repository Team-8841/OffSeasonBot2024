package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterSubsystem extends SubsystemBase {

    private CANSparkMax m_shooter = new CANSparkMax(15, MotorType.kBrushless);
    private CANSparkMax m_follower = new CANSparkMax(16, MotorType.kBrushless);

    private SparkPIDController m_shooterPID;
    private final RelativeEncoder m_encoder;
    private double m_kP = 0.000240;
    private double m_kI = 0.0000001;
    private double m_kD = 0;
    private double m_kIZone = 330;
    private double m_kFF = 0.000165;
    private double m_setPoint;
    private double m_allowedError = 200;


    public ShooterSubsystem() {
        configureSparkMax(m_shooter, IdleMode.kCoast, 40);
        configureSparkMax(m_follower, IdleMode.kCoast, 40);

        m_shooter.setInverted(true);
        m_follower.setInverted(false);

        m_follower.follow(m_shooter, true);

        m_shooterPID = m_shooter.getPIDController();
        m_encoder = m_shooter.getEncoder();


        m_shooterPID.setP(m_kP);
        m_shooterPID.setI(m_kI);
        m_shooterPID.setD(m_kD);
        m_shooterPID.setFF(m_kFF);
        m_shooterPID.setOutputRange(0, 1);

    }

    @Override
    public void periodic() {
        updateStatus();
    }  

    private void configureSparkMax(CANSparkMax spark, IdleMode idleMode, int currentLimit) {
        spark.restoreFactoryDefaults();
        spark.setSmartCurrentLimit(currentLimit);
        spark.setIdleMode(idleMode);
    }

    public void setShooterTargetSpeed(double rpm) {
        m_shooterPID.setReference(rpm, ControlType.kVelocity);
        m_setPoint = rpm;
    }

    public boolean upToSpeed() {
        double curVel = m_encoder.getVelocity();
        return curVel > m_setPoint - m_allowedError;
    }
 
    public void updateStatus() {
        SmartDashboard.putNumber("[Shooter]: SetPoint", m_setPoint);
        SmartDashboard.putNumber("[Shooter]: Velocity", m_encoder.getVelocity());
        SmartDashboard.putBoolean("[Shooter]: Ready", upToSpeed());
    }

}
