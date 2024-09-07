package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase {

    private CANSparkMax m_intake = new CANSparkMax(13, MotorType.kBrushed);
    private CANSparkMax m_index = new CANSparkMax(14, MotorType.kBrushed);

    private DigitalInput m_intakeSensor = new DigitalInput(0);
    private DigitalInput m_indexSensor = new DigitalInput(2);

    public IntakeSubsystem() {
        configureSparkMax(m_intake, IdleMode.kCoast, 40);
        configureSparkMax(m_index, IdleMode.kCoast, 40);

        m_intake.setInverted(false);
        m_index.setInverted(false);
    }
    

    private void configureSparkMax(CANSparkMax spark, IdleMode idleMode, int currentLimit) {
        spark.restoreFactoryDefaults();
        spark.setSmartCurrentLimit(currentLimit);
        spark.setIdleMode(idleMode);
    }


    public void setIntakeSpeed(double speed){
        m_intake.set(speed);
    }

    public void setIndexSpeed(double speed) {
        m_index.set(speed);
    }

    public boolean getIntakeSensor(){
        return m_intakeSensor.get();
    }

    public boolean getIndexSensor(){
        return !m_indexSensor.get();
    }

    public void SensorControl(boolean in, boolean out) {
        if (in) {
            if(!getIndexSensor()){
                if(!getIntakeSensor()){
                    setIntakeSpeed(0.9);
                    setIndexSpeed(0.40);
                } else if (getIntakeSensor()){
                    setIntakeSpeed(0.60);
                    setIndexSpeed(0.40);
                }
            }
            if (getIndexSensor()){
                setIndexSpeed(0);
                setIntakeSpeed(0);
            }
        } else if (out) {
            setIntakeSpeed(-0.7);
            setIndexSpeed(-0.7);
        } else {
            setIntakeSpeed(0);
            setIndexSpeed(0);
        }

    }
}
