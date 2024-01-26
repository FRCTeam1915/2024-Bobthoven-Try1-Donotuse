package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase {
    public static TalonSRX intake;
    public Intake() {
        intake = new TalonSRX(Constants.intake);
    }
}
