package frc.robot;

import com.ctre.phoenix6.hardware.CANrange;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import com.ctre.phoenix6.StatusSignal;



public class ElevatorSystem {
    private XboxController xboxControllerOne;

    // Elevator Speed Constants
    private double elevatorSpeed = 0.70;
    private double elevatorSpeedSlow = 0.05;

    // Elevator Height Position Constants
    private double ElevatorHeightMin = 0.00;
    private double elevatorPositionL2 = 365.00;
    private double elevatorPositionL3 = 1015.00;
    private double elevatorPositionL4 = 2150.00;

    // Head Speed Constants
    private double headSpeed = 0.50;
    private double headSpeedSlow = 0.10;

    // Intake Wheels Constant
    private double feedMotorSpeed = 0.80;

    // Universal OFF Constant
    private double off = 0.00;


    // Head Pivoting Angle Constants - MIN & MAX
    private double baseAngle = 115.00; //<-------------------------
    private double headAngleL4 = baseAngle - 44;
    private double headAngleL3 = baseAngle - 29;
    private double headAngleL2 = baseAngle - 29;
    private double headMaxOutAngle = baseAngle - 70; // baseAngle - 80;
    private double headDisabledAngle = baseAngle - 25;


    public ElevatorSystem(XboxController xboxControllerOne) {
        this.xboxControllerOne = xboxControllerOne;
    }



    // public static DutyCycleEncoder hexEncoderOne = new DutyCycleEncoder(0);
    public static DutyCycleEncoder headEncoder = new DutyCycleEncoder(1);
    
    final CANrange loadSensor = new CANrange(30);

    // Declare sparkmax's that control the elevator system
    final SparkMax ElevatorLeft = new SparkMax(40, MotorType.kBrushless);
    final SparkMax ElevatorRight = new SparkMax(41, MotorType.kBrushless);

    // declare relative encoders for elevator
    final RelativeEncoder ElevatorEncoderLeft = ElevatorLeft.getEncoder();
    final RelativeEncoder ElevatorEncoderRight = ElevatorRight.getEncoder();

    final SparkMax feedMotor = new SparkMax(42, MotorType.kBrushless);
    final SparkMax headPivotMotor = new SparkMax(43, MotorType.kBrushless);

    public void runHeadIntake() {
        feedMotor.set(feedMotorSpeed);
    }

    public void runHeadIntakeStop() {
        feedMotor.set(off);
    }

    public void runHeadIn(double headSpeed) {
        if (headEncoder.get() * 360 < baseAngle ) {
            headPivotMotor.set(-headSpeed);
        }
    }
    public void runHeadOut(double headSpeed) {
        if (headEncoder.get() * 360 > headMaxOutAngle ) {
            headPivotMotor.set(headSpeed);
        }
    }
    public void runHeadStop() {
        headPivotMotor.set(off);
    }

    public void runElevatorUp( double elevatorSpeed) {
        if (headEncoder.get() * 360 < headDisabledAngle) {
            if (ElevatorEncoderLeft.getPosition() * 360 < elevatorPositionL4) {
                ElevatorLeft.set(elevatorSpeed);
                ElevatorRight.set(-elevatorSpeed);
            }
        }
    }
    public void runElevatorDown(double elevatorSpeed) {
        if (headEncoder.get() * 360 < headDisabledAngle) {
            if (ElevatorEncoderLeft.getPosition() * 360 > ElevatorHeightMin) {
                ElevatorLeft.set(-elevatorSpeed);
                ElevatorRight.set(elevatorSpeed);
            }
        }
    }

    // Set the position of the elevator
    public void setElevatorPosition(double targetPosition) {
        if (headEncoder.get() * 360 < headDisabledAngle) {
            double posGate1 = targetPosition - 10;
            double posGate2 = targetPosition + 10;
            double speedGate1 = targetPosition - 50;
            double speedGate2 = targetPosition + 50;
            double currentPosition = ElevatorEncoderLeft.getPosition() * 360;

            if (currentPosition < speedGate1 || currentPosition > speedGate2) {
                if (currentPosition < posGate1) { runElevatorUp(elevatorSpeed); }
                else if (currentPosition > posGate2) { runElevatorDown(elevatorSpeed); } 
                else { runElevatorStop(); } 
            } else {
                if (currentPosition < posGate1) { runElevatorUp(elevatorSpeedSlow); }
                else if (currentPosition > posGate2) { runElevatorDown(elevatorSpeedSlow); } 
                else { runElevatorStop(); }
            }
        }

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
            if (currentAngle < angleGateOne) { runHeadIn(headSpeed); } 
            else if (currentAngle > angleGateTwo) { runHeadOut(headSpeed); }
            else { runHeadStop(); }

        } else {
            // if angle is within the speed gate range then slow down the headSpeed
            if (currentAngle < angleGateOne) { runHeadIn(headSpeedSlow); }
            else if (currentAngle > angleGateTwo) { runHeadOut(headSpeedSlow); }
            else { runHeadStop(); }
        }

    }

    // Add Controls now...
    public void update() {

        StatusSignal<Distance> distanceSignal = loadSensor.getDistance();
        distanceSignal.refresh();

        double headSensorValue = distanceSignal.getValueAsDouble()*1000;
        double ElevatorPositionLeft = ElevatorEncoderLeft.getPosition() * 360;
        double ElevatorPositionRight = ElevatorEncoderRight.getPosition() * 360;
        double headAngle = headEncoder.get() *360;

        SmartDashboard.putNumber("Head Angle: ", headAngle);
        SmartDashboard.putNumber("ElevatorPosLeft: ", ElevatorPositionLeft);
        SmartDashboard.putNumber("ElevatorPosRight: ", ElevatorPositionRight);
        SmartDashboard.putNumber("LoadSensor: ", headSensorValue); // If less than 85 then pipe is inserted.


        // Intake wheel functionality
        if (xboxControllerOne.getRightTriggerAxis() > 0.50) {
            feedMotor.set(feedMotorSpeed);
        }
        else if ((xboxControllerOne.getLeftTriggerAxis() > 0.50)) {
            feedMotor.set(-feedMotorSpeed);
        } else {
            feedMotor.set(off);
        }


        // Set head to L2 position
        if (xboxControllerOne.getAButton()) {
            commandHeadAngle(headAngleL2);          // Move head out
            commandSetHeight(elevatorPositionL2);   // Set elevator position
            commandHeadAngle(headAngleL2);          // Move head to L2 angle

        }
        // Set head to L3 position
        else if (xboxControllerOne.getBButton()) {
            commandHeadAngle(headAngleL3);          // Move head out
            commandSetHeight(elevatorPositionL3);   // Set elevator position
            commandHeadAngle(headAngleL3);          // Move head to L2 angle

        }
        // Set head to L4 position
        else if (xboxControllerOne.getYButton()) {
            commandHeadAngle(headAngleL4);          // Move head out
            commandSetHeight(elevatorPositionL4);   // Set elevator position
            commandHeadAngle(headAngleL4);          // Move head to L2 angle

        }
        // Set head to load position
        else if (xboxControllerOne.getXButton()) {
            commandSetHeight(ElevatorHeightMin);   // Set elevator position
            commandHeadAngle(baseAngle);          // Move head to L2 angle
        }



        // Sensor functionality
        if (xboxControllerOne.getRightBumperButton()) {
            if (headSensorValue > 85) {
                feedMotor.set(feedMotorSpeed);
            } else {
                feedMotor.set(off);
            }
        }


        // Run button Command
        if (xboxControllerOne.getLeftBumperButton()) {

            // Elevator Joystick pushed forward
            if (xboxControllerOne.getLeftY() < -0.50) {
                runElevatorUp(elevatorSpeed);
            }

            // Elevator Joystick pushed backwards
            else if (xboxControllerOne.getLeftY() > 0.50) {
                runElevatorDown(elevatorSpeed);
            }

            else if (xboxControllerOne.getLeftStickButton()) {
                ElevatorLeft.set(-elevatorSpeedSlow);
                ElevatorRight.set(elevatorSpeedSlow);
            }

            // When the joystick is not active then stop the elevator
            else {
                runElevatorStop();
            }

            // HEAD Joystick pushed forward
            if (xboxControllerOne.getRightY() < -0.50) {
                setHeadAngle(headMaxOutAngle);
            }
            // HEAD Joystick pushed backward
            else if (xboxControllerOne.getRightY() > 0.50) {
                setHeadAngle(baseAngle);
            }
            else {
                runHeadStop();
            }

            // Zero the elevator encoder.
            if (xboxControllerOne.getBackButtonPressed()) {
                ElevatorEncoderLeft.setPosition(0.00);
                ElevatorEncoderRight.setPosition(0.00);
            }

        }

        // Make head move really slow without any angle restriction.
        // if (xboxControllerOne.getRightStickButton()) {
        //     headPivotMotor.set(-0.05);
        // } else {
        //     headPivotMotor.set(off);
        // }


    }




    // Moves head out to certain angle.
    public void commandRunIntake(double seconds) {
        Timer timer = new Timer();
        boolean loaded = false;
        while (timer.get() < seconds && loaded == false) {
            System.out.println(timer.get());
            runHeadIntake();
        }
        runHeadIntakeStop();
    }

    // Moves head out to certain angle.
    public void commandHeadAngle(double headAngle) {
        Timer timer = new Timer();
        double currentAngle = headEncoder.get() * 360;
        double gate1 = headAngle - 1;
        double gate2 = headAngle + 1;
        while (timer.get() < 5.00 && currentAngle < gate1 || currentAngle > gate2) {
            setHeadAngle(headAngle);
            currentAngle = headEncoder.get() *360;
        }
        runHeadStop();
    }

    public void commandSetHeight(double height) { // Set to specific angle, should stop when it hits the angle.
        Timer timer = new Timer();
        double currentHeight = ElevatorEncoderLeft.getPosition() * 360;
        double gate1 = height - 10;
        double gate2 = height + 10;
        while (timer.get() < 5.00 && currentHeight < gate1 || currentHeight > gate2) {
            setElevatorPosition(height);
            currentHeight = ElevatorEncoderLeft.getPosition() * 360;
        }
        runElevatorStop();
    }






}



//FUTURE ADDITION
// Steps...

// 1. Head go out to certain position
// 2. Elevator go up to certain button position
// 3. Adjust head for the new angle
// 4. (Move forward if needed?)
// 5. Run head wheels to expel the pipe onto the Reef