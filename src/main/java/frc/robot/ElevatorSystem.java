package frc.robot;

import com.revrobotics.spark.SparkMax;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase; // ✅ Import SubsystemBase

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;

public class ElevatorSystem extends SubsystemBase { // ✅ Extend SubsystemBase

    // Elevator Speed Constants
    public double elevatorSpeed = 0.70;
    public double elevatorSpeedSlow = 0.05;

    // Elevator Height Position Constants
    public double ElevatorHeightMin = 0.00;
    public double elevatorPositionL2 = 570.00;
    public double elevatorPositionL3 = 1220.00;
    public double elevatorPositionL4 = 2180.00;

    public double elevatorClimbPosition = 610.00;


    // Universal OFF Constant
    public double off = 0.00;

    // Declare SparkMax motors
    public final SparkMax ElevatorLeft = new SparkMax(40, MotorType.kBrushless);
    public final SparkMax ElevatorRight = new SparkMax(41, MotorType.kBrushless);

    // Declare relative encoders
    public final RelativeEncoder ElevatorEncoderLeft = ElevatorLeft.getEncoder();
    public final RelativeEncoder ElevatorEncoderRight = ElevatorRight.getEncoder();


    public void runElevatorUp(double speed) {
        if (Robot.getInstance().m_robotContainer.HeadSystem.headEnabled) {
            if (ElevatorEncoderLeft.getPosition() * 360 < elevatorPositionL4) {
                ElevatorLeft.set(speed);
                ElevatorRight.set(-speed);
            }
        }
    }

    public void runElevatorDown(double speed) {
        if (Robot.getInstance().m_robotContainer.HeadSystem.headEnabled) {
            if (ElevatorEncoderLeft.getPosition() * 360 > ElevatorHeightMin) {
                ElevatorLeft.set(-speed);
                ElevatorRight.set(speed);
            }
        }
    }

    public void setElevatorPosition(double targetPosition) {
        double currentPosition = ElevatorEncoderLeft.getPosition() * 360;
        double posGate1 = targetPosition - 10;
        double posGate2 = targetPosition + 10;
        double speedGate1 = targetPosition - 50;
        double speedGate2 = targetPosition + 50;

        if (currentPosition < speedGate1 || currentPosition > speedGate2) {
            if (currentPosition < posGate1) {
                runElevatorUp(elevatorSpeed);
            } else if (currentPosition > posGate2) {
                runElevatorDown(elevatorSpeed);
            } else {
                runElevatorStop();
            }
        } else {
            if (currentPosition < posGate1) {
                runElevatorUp(elevatorSpeedSlow);
            } else if (currentPosition > posGate2) {
                runElevatorDown(elevatorSpeedSlow);
            } else {
                runElevatorStop();
            }
        }
    }

    public void runElevatorStop() {
        ElevatorLeft.set(off);
        ElevatorRight.set(off);
    }
    public void runElevatorReset() {
        ElevatorEncoderLeft.setPosition(0.00);
        ElevatorEncoderRight.setPosition(0.00);
    }

    @Override
    public void periodic() {  // ✅ WPILib calls this every loop
        SmartDashboard.putNumber("ElevatorPosLeft", ElevatorEncoderLeft.getPosition() * 360);
        SmartDashboard.putNumber("ElevatorPosRight", ElevatorEncoderRight.getPosition() * 360);
    }
}
