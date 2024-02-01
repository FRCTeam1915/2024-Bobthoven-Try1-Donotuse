package frc.robot.commands;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Intake;

public class shooter extends CommandBase{
    Intake upperMotor;
    boolean in;
    public shooter(Intake upperMotor2, boolean tin){
        in = tin;
        upperMotor = upperMotor2;
        addRequirements(upperMotor);
    }

    @Override
    public void execute(){
        Intake.intake.set(ControlMode.PercentOutput, -1);
    }
}
