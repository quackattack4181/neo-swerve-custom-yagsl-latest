package frc.robot;

import com.ctre.phoenix6.hardware.CANrange;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;

public class ElevatorSystem {

    private XboxController xboxControllerOne;

    private double elevatorSpeed = 0.40;
    private double headSpeed = 0.40;

    private double headSpeedSlow = 0.10;
    private double feedMotorSpeed = 0.80;

    private double off = 0.00;

    private double disableElevatorAngle = 275;


    // Head Pivot Encoder #2 - MIN & MAX
    private double baseAngle = 251.00;
    private double angleL4 = baseAngle + 53;
    private double angleL3 = baseAngle + 19;
    private double angleL2 = baseAngle + 19;

    private double ElevatorHeightMax = 2220.00;

    // Level 4 Head angle should be 305.
    // Level 2 & 3 Head angle should be 286.
    // Loading Level should be 276




    public ElevatorSystem(XboxController xboxControllerOne) {
        this.xboxControllerOne = xboxControllerOne;
    }

    // public static DutyCycleEncoder hexEncoderOne = new DutyCycleEncoder(0);
    public static DutyCycleEncoder headEncoder = new DutyCycleEncoder(1);
    public static DutyCycleEncoder hexEncoderThree = new DutyCycleEncoder(2);
    
    final CANrange headSensor = new CANrange(30);

    // Declare sparkmax's that control the elevator system
    final SparkMax ElevatorLeft = new SparkMax(40, MotorType.kBrushless);
    final SparkMax ElevatorRight = new SparkMax(41, MotorType.kBrushless);

    // declare relative encoders for elevator
    final RelativeEncoder ElevatorEncoderLeft = ElevatorLeft.getEncoder();
    final RelativeEncoder ElevatorEncoderRight = ElevatorRight.getEncoder();

    final SparkMax feedMotor = new SparkMax(42, MotorType.kBrushless);
    final SparkMax headPivotMotor = new SparkMax(43, MotorType.kBrushless);

    public void runHeadIn(double headSpeed) {
        headPivotMotor.set(headSpeed);
    }
    public void runHeadOut(double headSpeed) {
        headPivotMotor.set(-headSpeed);
    }
    public void runHeadStop() {
        headPivotMotor.set(off);
    }

    public void runElevatorUp() {
        ElevatorLeft.set(elevatorSpeed);
        ElevatorRight.set(-elevatorSpeed);
    }
    public void runElevatorDown() {
        ElevatorLeft.set(-elevatorSpeed);
        ElevatorRight.set(elevatorSpeed);
    }
    public void runElevatorStop() {
        ElevatorLeft.set(off);
        ElevatorRight.set(off);
    }

    // Set head angle to given target angle
    public void setHeadAngle(double targetAngle) {
        double currentAngle = headEncoder.get() * 360;

        double speedGateOne = targetAngle -8;
        double speedGateTwo = targetAngle +8;
        double angleGateOne = targetAngle -1;
        double angleGateTwo = targetAngle +1;


        if (currentAngle < speedGateOne || currentAngle > speedGateTwo) { 
            if (currentAngle < angleGateOne) {
                runHeadOut(headSpeed);
    
            } else if (currentAngle > angleGateTwo) {
                runHeadIn(headSpeed);
            }
            else {
                runHeadStop();
            }

        } else {
            // if angle is within the speed gate range then slow down the headSpeed
            if (currentAngle < angleGateOne) {
                runHeadOut(headSpeedSlow);
    
            } else if (currentAngle > angleGateTwo) {
                runHeadIn(headSpeedSlow);
            }
            else {
                runHeadStop();
            }
        }

    }

    // Add Controls now...
    public void update() {
        double ElevatorPositionLeft = ElevatorEncoderLeft.getPosition() * 360;
        double ElevatorPositionRight = ElevatorEncoderRight.getPosition() * 360;
        double headAngle = headEncoder.get() *360;

        SmartDashboard.putNumber("Head Angle: ", headAngle);
        SmartDashboard.putNumber("ElevatorPosLeft: ", ElevatorPositionLeft);
        SmartDashboard.putNumber("ElevatorPosRight: ", ElevatorPositionRight);




        // HEAD Joystick pushed forward
        if (xboxControllerOne.getRightY() < -0.50) {
            runHeadOut(headSpeed);
        }
        // HEAD Joystick pushed backward
        else if (xboxControllerOne.getRightY() > 0.50) {
            runHeadIn(headSpeed);
        }
        else {
            runHeadStop();
        }


        if (xboxControllerOne.getRightTriggerAxis() > 0.50) {
            feedMotor.set(feedMotorSpeed);
        }
        else if ((xboxControllerOne.getLeftTriggerAxis() > 0.50)) {
            feedMotor.set(-feedMotorSpeed);
        } else {
            feedMotor.set(off);
        }



        // If statement to stop the elevator from moving if the head isn't far enough out
        if (headEncoder.get() * 360 > disableElevatorAngle) {

            // Elevator Joystick pushed forward
            if (xboxControllerOne.getLeftY() < -0.50) {
                runElevatorUp();
            }

            // Elevator Joystick pushed backwards
            else if (xboxControllerOne.getLeftY() > 0.50) {
                runElevatorDown();
            }

            // When the joystick is not active then stop the elevator
            else {
                runElevatorStop();
            }
        }


        if (xboxControllerOne.getBackButtonPressed()) {
            ElevatorEncoderLeft.setPosition(0.00);
            ElevatorEncoderRight.setPosition(0.00);
        }

        if (xboxControllerOne.getBButton()) {
            setHeadAngle(angleL3);
        }
        if (xboxControllerOne.getYButton()) {
            setHeadAngle(angleL4);
        }
        if (xboxControllerOne.getXButton()) {
            setHeadAngle(baseAngle);
        }




    }

}
