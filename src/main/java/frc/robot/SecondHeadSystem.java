package frc.robot;

import com.revrobotics.spark.SparkMax;
import edu.wpi.first.wpilibj2.command.SubsystemBase; // ✅ Import SubsystemBase
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.revrobotics.spark.SparkLowLevel.MotorType;

public class SecondHeadSystem extends SubsystemBase { // ✅ Renamed & Extended SubsystemBase
    // Intake Wheels Constant
    private final double feedMotorSpeed = 0.80;

    // Head pivot motor speed
    private final double headPivotSpeed = 0.15;
    private final double headPivotSpeedMedium = 0.10;
    private final double headPivotSpeedSlow = 0.05;

    
    // Universal OFF Constant
    private final double off = 0.00;
    
    // Head Pivoting Angle Constants - MIN & MAX
    private final double baseAngleIn = 330.00;
    private final double ChoralShootAngle = 320.00;
    private final double angleShoot = baseAngleIn - 40;
    private final double angleOut = baseAngleIn - 80;
    
    // Default is feed motors off and put at baseAngleIn position.
    // Left trigger puts head at angleOut and runs feed motors
    // Left Bumper brings to shoot position.
    // Right trigger will shoot the pipe out with feed motors
    // Bumper puts head to shooting angle when you let go it shoots it and then goes back to baseAngle
    
    // Declare motors
    public final SparkMax headPivotMotor = new SparkMax(51, MotorType.kBrushless);
    public final SparkMax feedMotorOne = new SparkMax(55, MotorType.kBrushless);
    public final SparkMax feedMotorTwo = new SparkMax(52, MotorType.kBrushless);
    
    public final DutyCycleEncoder secondHeadEncoder = new DutyCycleEncoder(2);
    public double secondHeadAngle = secondHeadEncoder.get() * 360;

    public void runHeadIntake() {
        feedMotorOne.set(-feedMotorSpeed);
        feedMotorTwo.set(feedMotorSpeed);
    }

    public void runHeadIntakeReverse() {
        feedMotorOne.set(feedMotorSpeed);
        feedMotorTwo.set(-feedMotorSpeed);
    }

    public void runHeadIntakeStop() {
        feedMotorOne.set(off);
        feedMotorTwo.set(off);
    }

    public void runHeadOut() {
        headPivotMotor.set(-headPivotSpeedSlow);
    }

    public void runHeadIn() {
        headPivotMotor.set(headPivotSpeed);
    }

    public void runHeadStop() {
        headPivotMotor.set(off);
    }

    public void setSecondHeadAngle(double targetAngle) {
        double currentAngle = secondHeadEncoder.get() * 360;
        double angleGateOne = targetAngle - 3;
        double angleGateTwo = targetAngle + 3;

        if (currentAngle < angleGateOne) {
            runHeadIn(); // Moves head upwards toward target
        } else if (currentAngle > angleGateTwo) {
            runHeadOut(); // Moves head downwards toward target
        }
        else {
            runHeadStop(); // Stops if within range
        }
    }


    @Override
    public void periodic() { // ✅ WPILib calls this every robot loop (~20ms)
        secondHeadAngle = secondHeadEncoder.get() * 360;
        SmartDashboard.putNumber("SecondHeadAngle", secondHeadAngle);
    }
}
