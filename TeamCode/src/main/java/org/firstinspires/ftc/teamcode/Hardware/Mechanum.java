package org.firstinspires.ftc.teamcode.Hardware;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Movement.Location;
import org.firstinspires.ftc.teamcode.Hardware.Robot.*;

public class Mechanum extends Robot{

    /**
     * Constructor 1
     *
     * @param loc
     */
    public Mechanum(Location loc, HardwareMap hw) {
        super(RobotType.DIFFY_MECH, loc,hw);
    }
}
