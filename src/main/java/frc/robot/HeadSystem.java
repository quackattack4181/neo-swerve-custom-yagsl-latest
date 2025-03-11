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
    public double primaryHeadSpeed = Constants.CustomConstants.primaryHeadSpeed;
    public double primaryHeadSpeedSlow = Constants.CustomConstants.primaryHeadSpeedSlow;

    // Intake Wheels Constant
    public double feedMotorSpeed = Constants.CustomConstants.feedMotorSpeed;

    // Universal OFF Constant
    public double off = 0.00;

    // Head Pivoting Angle Constants - MIN & MAX   106
    public double baseAngle = Constants.CustomConstants.baseAngle;
    public double headAngleL4 = Constants.CustomConstants.headAngleL4;
    public double headAngleL3 = Constants.CustomConstants.headAngleL3;
    public double headAngleL2 = Constants.CustomConstants.headAngleL2;
    public double headMaxOutAngle = Constants.CustomConstants.headMaxOutAngle;
    public double headDisabledAngle = Constants.CustomConstants.headDisabledAngle;

    public boolean headEnabled = false;

    public DutyCycleEncoder headEncoder = new DutyCycleEncoder(4);

    public final SparkMax feedMotor = new SparkMax(42, MotorType.kBrushless);
    public final SparkMax headPivotMotor = new SparkMax(43, MotorType.kBrushless);
    public final CANrange loadSensor = new CANrange(30);

    public void runHeadIntake() {
        feedMotor.set(feedMotorSpeed);
    }

    public void runHeadIntakeStop() {
        feedMotor.set(off);
    }

    public void runHeadIn(double primaryHeadSpeed) {
        if (headEncoder.get() * 360 < baseAngle) {
            headPivotMotor.set(primaryHeadSpeed);
            
        }
    }

    public void runHeadOut(double primaryHeadSpeed) {
        if (headEncoder.get() * 360 > headMaxOutAngle) {
            headPivotMotor.set(-primaryHeadSpeed);
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
                runHeadIn(primaryHeadSpeed);
            } else if (currentAngle > angleGateTwo) {
                runHeadOut(primaryHeadSpeed);
            } else {
                runHeadStop();
            }
        } else {
            if (currentAngle < angleGateOne) {
                runHeadIn(primaryHeadSpeedSlow);
            } else if (currentAngle > angleGateTwo) {
                runHeadOut(primaryHeadSpeedSlow);
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
