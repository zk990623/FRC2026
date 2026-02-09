package frc.robot.intake;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SimpleMotorSubsystem extends SubsystemBase {
    private final SparkMax neoMotor;

    public SimpleMotorSubsystem() {

        this.neoMotor = new SparkMax(9, MotorType.kBrushless);
    }

    public Command spinCommand(double speed) {

        return this.runEnd(
            () -> neoMotor.set(speed), 
            () -> neoMotor.set(0)
        );
    }
}