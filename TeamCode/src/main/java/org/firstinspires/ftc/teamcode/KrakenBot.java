package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.*;
import com.qualcomm.robotcore.util.*;

/**
 * Created by eriche on 10/28/17.
 */

public class KrakenBot {
    // Drive Motors
    public DcMotor left_drive;
    public DcMotor right_drive;

    // Arm Motors
    public DcMotor lower_arm;
    public DcMotor upper_arm;

    // Claw Motors
    public Servo wrist;
    public Servo left_claw;
    public Servo right_claw;

    public static final double CLAW_HOME=0.2;
    public static final double INV_CLAW_HOME=0.8;
    public static final double WRIST_HOME=0.3;

    HardwareMap hwMap = null;
    private ElapsedTime period = new ElapsedTime();
}
