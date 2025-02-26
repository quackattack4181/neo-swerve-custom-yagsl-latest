package frc.robot;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.CANrange;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase; // ✅ Import SubsystemBase

public class HeadSystem extends SubsystemBase { // ✅ Extend SubsystemBase

    // Head Speed Constants
    public double headSpeed = 0.50;
    public double headSpeedSlow = 0.10;

    // Intake Wheels Constant
    public double feedMotorSpeed = 0.80;

    // Universal OFF Constant
    public double off = 0.00;

    // Head Pivoting Angle Constants - MIN & MAX
    public double baseAngle = 115.00;
    public double headAngleL4 = baseAngle - 20;
    public double headAngleL3 = baseAngle - 29;
    public double headAngleL2 = baseAngle - 29;
    public double headMaxOutAngle = baseAngle - 70;
    public double headDisabledAngle = baseAngle - 15;

    public boolean headEnabled = false;

    public DutyCycleEncoder headEncoder = new DutyCycleEncoder(1);

    public final SparkMax feedMotor = new SparkMax(42, MotorType.kBrushless);
    public final SparkMax headPivotMotor = new SparkMax(43, MotorType.kBrushless);
    public final CANrange loadSensor = new CANrange(30);

    public void runHeadIntake() {
        feedMotor.set(feedMotorSpeed);
    }

    public void runHeadIntakeStop() {
        feedMotor.set(off);
    }

    public void runHeadIn(double headSpeed) {
        if (headEncoder.get() * 360 < baseAngle) {
            headPivotMotor.set(-headSpeed);
        }
    }

    public void runHeadOut(double headSpeed) {
        if (headEncoder.get() * 360 > headMaxOutAngle) {
            headPivotMotor.set(headSpeed);
        }
    }

    public void runHeadStop() {
        headPivotMotor.set(off);
    }

    public void setHeadAngle(double targetAngle) {
        double currentAngle = headEncoder.get() * 360;
        double speedGateOne = targetAngle - 8;
        double speedGateTwo = targetAngle + 8;
        double angleGateOne = targetAngle - 1;
        double angleGateTwo = targetAngle + 1;

        if (currentAngle < speedGateOne || currentAngle > speedGateTwo) {
            if (currentAngle < angleGateOne) {
                runHeadIn(headSpeed);
            } else if (currentAngle > angleGateTwo) {
                runHeadOut(headSpeed);
            } else {
                runHeadStop();
            }
        } else {
            if (currentAngle < angleGateOne) {
                runHeadIn(headSpeedSlow);
            } else if (currentAngle > angleGateTwo) {
                runHeadOut(headSpeedSlow);
            } else {
                runHeadStop();
            }
        }
    }

    @Override
    public void periodic() { // ✅ Called every robot loop cycle (~20ms)
        StatusSignal<Distance> distanceSignal = loadSensor.getDistance();
        distanceSignal.refresh();

        double headAngle = headEncoder.get() * 360;
        double headSensorValue = distanceSignal.getValueAsDouble() * 1000;

        SmartDashboard.putNumber("LoadSensor", headSensorValue);
        SmartDashboard.putNumber("Head Angle", headAngle);

        headEnabled = headAngle < headDisabledAngle;
    }
}
