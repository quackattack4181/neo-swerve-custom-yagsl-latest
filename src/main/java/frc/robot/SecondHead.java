package frc.robot;

import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Commands;

import com.revrobotics.spark.SparkLowLevel.MotorType;

public class SecondHead {
    private XboxController xboxController;

    // Intake Wheels Constant
    private double feedMotorSpeed = 0.80;

    // Head pivot motor speed
    private double headPivotSpeed = 0.20;
    private double headPivotSpeedSlow = 0.05;

    // Universal OFF Constant
    private double off = 0.00;


    // Head Pivoting Angle Constants - MIN & MAX
    private double baseAngle = 115.00; //<-------------------------
    private double headAngleUp = baseAngle - 44;
    private double headAngleDown = baseAngle - 29;



    public SecondHead(XboxController xboxController) {
        this.xboxController = xboxController;
    }

    final SparkMax headPivotMotor = new SparkMax(51, MotorType.kBrushless);

    final SparkMax feedMotorOne = new SparkMax(55, MotorType.kBrushless);
    final SparkMax feedMotorTwo = new SparkMax(52, MotorType.kBrushless);

    public static DutyCycleEncoder secondHeadEncoder = new DutyCycleEncoder(3);


    // Runs the second head intake
    public void runHeadIntake() {
        feedMotorOne.set(-feedMotorSpeed);
        feedMotorTwo.set(feedMotorSpeed);
    }

    // Runs the second head intake in reverse
    public void runHeadIntakeReverse() {
        feedMotorOne.set(feedMotorSpeed);
        feedMotorTwo.set(-feedMotorSpeed);
    }

    // Stops the second head intake wheels
    public void runHeadIntakeStop() {
        feedMotorOne.set(off);
        feedMotorTwo.set(off);
    }

    public void runHeadDown() {
        headPivotMotor.set(-headPivotSpeedSlow);
    }

    public void runHeadUp() {
        headPivotMotor.set(headPivotSpeed);
    }

    public void runHeadStop() {
        headPivotMotor.set(off);
    }

    public void runHeadHold() {
        headPivotMotor.set(0.03);
    }


    // A = Suck in ball     runHeadIntake();
    // X = Push ball out    runHeadIntakeReverse();
    // Y = Head up          runHeadDown();
    // B = Head down        runHeadUp();


    // Add Controls now...
    public void update() {

        double headAngle = secondHeadEncoder.get() * 360;

        // Runs the second head intake motors
        if (xboxController.getRightBumperButton()) {
            runHeadIntake();
        } else if (xboxController.getRightTriggerAxis() > 0.50) {
            runHeadIntakeReverse();
        } else {
            runHeadIntakeStop();
        }


        // Runs the second head pivot motor
        if (xboxController.getLeftBumperButton()) {
            runHeadUp();
        } else if (xboxController.getLeftTriggerAxis() > 0.50) {
            runHeadDown();
        } else {
            runHeadStop();
        }


    }

}
