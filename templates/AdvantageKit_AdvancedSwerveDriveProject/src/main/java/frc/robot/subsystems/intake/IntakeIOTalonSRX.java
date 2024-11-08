package frc.robot.subsystems.intake;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.RobotController;

/** Example hardware implementation of IntakeIO using 775pro with talon srx and a beam breaker sensor */
public class IntakeIOTalonSRX implements IntakeIO {
    private final TalonSRX intake775 = new TalonSRX(24);
    private final DigitalInput intakeBeamBreaker = new DigitalInput(4);

    @Override
    public void updateInputs(IntakeInputs inputs) {
        // detect the note with the beam breaker and store it in "inputs"
        // if the note is present, the beam breaker is blocked and will return "false"
        inputs.noteDetected = !intakeBeamBreaker.get();
    }

    @Override
    public void runIntakeVoltage(double volts) {
        // run the voltage on the intake 775
        intake775.set(ControlMode.PercentOutput, volts / RobotController.getBatteryVoltage());
    }
}
