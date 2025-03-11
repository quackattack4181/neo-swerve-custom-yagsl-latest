package frc.robot;


import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase; // ✅ Import SubsystemBase

public class ClimberSystem extends SubsystemBase { // ✅ Extend SubsystemBase

    // Climber Speed Constants
    public double climbSpeed = Constants.CustomConstants.climbSpeed;
    public double off = 0.00;
    public double climbCurrentAngle;

    // Outward final value is -300.
    // Inward final value should be 0.0

    public final SparkMax climbMotor = new SparkMax(51, MotorType.kBrushless);

    // Declare relative encoders
    public final RelativeEncoder climbMotorEncoder = climbMotor.getEncoder();


    public void setClimbPosition(double targetPosition) {
        double currentPosition = climbMotorEncoder.getPosition();
    
        if (currentPosition < targetPosition - 1) {
            runClimbArmInward();  // Move inward if below target
        } else if (currentPosition > targetPosition + 1) {
            runClimbArmOutward(); // Move outward if above target
        } else {
            runClimbArmStop();    // Stop if within 1 unit of target
        }
    }
    

    public void runClimbArmInward() { // encoder value gets larger...
        climbMotor.set(climbSpeed);
        
    }

    public void runClimbArmOutward() { // encoder value gets smaller...
        climbMotor.set(-climbSpeed);
    }

    public void runClimbArmStop() {
        climbMotor.set(off);
    }

    @Override
    public void periodic() { // ✅ Called every robot loop cycle (~20ms)
        SmartDashboard.putNumber("ClimbMotorValue", climbMotorEncoder.getPosition());

    }
}
