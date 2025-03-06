package frc.robot;


import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.wpilibj2.command.SubsystemBase; // ✅ Import SubsystemBase

public class ClimberSystem extends SubsystemBase { // ✅ Extend SubsystemBase

    // Head Speed Constants
    public double climbSpeed = 0.90;
    public double off = 0.00;

    public final SparkMax climbMotor = new SparkMax(51, MotorType.kBrushless);

    public void runClimbArmInward() {
        climbMotor.set(climbSpeed);
    }

    public void runClimbArmOutward() {
        climbMotor.set(-climbSpeed);
    }

    public void runClimbArmStop() {
        climbMotor.set(off);
    }

    @Override
    public void periodic() { // ✅ Called every robot loop cycle (~20ms)

    }
}
