package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase; // ✅ Import SubsystemBase
import frc.robot.Constants;
import frc.robot.Robot;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;

public class ElevatorSystem extends SubsystemBase { // ✅ Extend SubsystemBase

    // Elevator Speed Constants
    public double elevatorSpeed = Constants.CustomConstants.elevatorSpeed;
    public double elevatorSpeedSlow = Constants.CustomConstants.elevatorSpeedSlow;

    // Elevator Height Position Constants
    public double ElevatorHeightMin = Constants.CustomConstants.ElevatorHeightMin;
    public double elevatorPositionL2 = Constants.CustomConstants.elevatorPositionL2;
    public double elevatorPositionL3 = Constants.CustomConstants.elevatorPositionL3;
    public double elevatorPositionL4 = Constants.CustomConstants.elevatorPositionL4;



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
