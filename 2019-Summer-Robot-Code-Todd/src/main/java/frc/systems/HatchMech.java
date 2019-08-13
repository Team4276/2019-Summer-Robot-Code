/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.systems;

import frc.robot.Robot;
import frc.utilities.Toggler;
import frc.utilities.Xbox;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class HatchMech {
    private DoubleSolenoid hatchSolenoid;
    Toggler hatch;
    double activateTime = 0.25;
    public boolean isEjecting = false;
    private final Value kApart = DoubleSolenoid.Value.kReverse;
    private final Value kTogether = DoubleSolenoid.Value.kForward;

    public HatchMech(int fwdSolenoidPort, int revSolenoidPort) {
        hatchSolenoid = new DoubleSolenoid(fwdSolenoidPort, revSolenoidPort);
        hatch = new Toggler(Xbox.LB);
    }

    public void performMainProcessing() {
        hatch.updateMechanismState();
        boolean togglerState = hatch.getMechanismState();
        if (togglerState) {
            Robot.xboxJoystick.setRumble(RumbleType.kRightRumble, 0.3);
            actuateSolenoid();
        } else {
            Robot.xboxJoystick.setRumble(RumbleType.kRightRumble, 0.0);
            deactuateSolenoid();
        }
        updateTelemetry();
    }

    public void actuateSolenoid() {
        isEjecting = true;
        hatchSolenoid.set(kTogether);
        // ejectTime.setTimer(activateTime);

    }

    public void deactuateSolenoid() {
        isEjecting = false;
        hatchSolenoid.set(kApart);

    }

    public void updateTelemetry() {
        SmartDashboard.putBoolean("Ejecting?", isEjecting);
    }
}