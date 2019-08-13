package frc.systems;

import frc.robot.Robot;
import frc.utilities.Xbox;
import frc.utilities.Toggler;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.ctre.phoenix.motorcontrol.ControlMode;

public class Arm {

    VictorSPX left, right;

    DoubleSolenoid armSolenoid;

    Toggler arm;

    private final Value kDown = DoubleSolenoid.Value.kReverse;
    private final Value kUp = DoubleSolenoid.Value.kForward;

    boolean isUp = true;

    double rightIntakeSpeed = 1;
    double rightOuttakeSpeed = -0.3;

    double leftIntakeSpeed = 1;
    double leftOuttakeSpeed = -0.3;

    boolean isCollecting = false;

    public Arm(int rightCANPort, int leftCANPort, int fwdSolenoidPort, int revSolenoidPort) {
        left = new VictorSPX(leftCANPort);
        right = new VictorSPX(rightCANPort);
        armSolenoid = new DoubleSolenoid(fwdSolenoidPort, revSolenoidPort);
        arm = new Toggler(Xbox.LB);
    }

    public void intake() {
        left.set(ControlMode.PercentOutput, leftIntakeSpeed);
        right.set(ControlMode.PercentOutput, rightIntakeSpeed);
        isCollecting = true;
    }

    public void outtake() {
        left.set(ControlMode.PercentOutput, leftOuttakeSpeed);
        right.set(ControlMode.PercentOutput, rightOuttakeSpeed);
    }

    public void stop() {
        left.set(ControlMode.PercentOutput, 0);
        right.set(ControlMode.PercentOutput, 0);
        isCollecting = false;
    }

    public void actuateArmSolenoid() {
        isUp = true;
        armSolenoid.set(kUp);
        // ejectTime.setTimer(activateTime);
    }

    public void deactuateArmSolenoid() {
        isUp = false;
        armSolenoid.set(kDown);

    }

    public void performMainProcessing() {
        if (Robot.xboxJoystick.getRawButton(Xbox.RT)) {
            intake();
        } else if (Robot.xboxJoystick.getRawButton(Xbox.LT)) {
            outtake();
        } else {
            stop();
        }

        arm.updateMechanismState();
        boolean togglerState = arm.getMechanismState();
        if (togglerState){
            actuateArmSolenoid();;
        } else {
            deactuateArmSolenoid();
        }
        updateTelemetry();
    }

    public void updateTelemetry() {
        SmartDashboard.putBoolean("Collecting", isCollecting);
        SmartDashboard.putBoolean("Arm State", isUp);
    }

}