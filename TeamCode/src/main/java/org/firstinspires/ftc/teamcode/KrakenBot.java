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

    // Constants
    public static final double CLAW_HOME=0.8;
    public static final double INV_CLAW_HOME=0.2;
    public static final double WRIST_HOME=1;

    public static final double MID_SERVO       =  0.5 ;
    public static final double ARM_UP_POWER    =  0.2 ;
    public static final double ARM_DOWN_POWER  = -0.2 ;

    // Local OpMode members
    HardwareMap hwMap = null;
    private ElapsedTime period = new ElapsedTime();

    // Constructor
    public KrakenBot() {}

    // Initialize hardware
    public void init(HardwareMap ahwMap) {
        // save reference to HW Map
        hwMap = ahwMap;

        // Define and Initialize Motors
        left_drive  = hwMap.get(DcMotor.class, "left_drive");
        right_drive = hwMap.get(DcMotor.class, "right_drive");
        upper_arm  = hwMap.get(DcMotor.class, "upper_arm");
        lower_arm = hwMap.get(DcMotor.class, "lower_arm");
        left_drive.setDirection(DcMotor.Direction.REVERSE);

        // Set all motors to zero power
        left_drive.setPower(0);
        right_drive.setPower(0);
        lower_arm.setPower(0);
        upper_arm.setPower(0);

        // Set drive motors to run with encoders
        left_drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        right_drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lower_arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        upper_arm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Define and initialize ALL installed servos.
        wrist = hwMap.get(Servo.class, "wrist");
        left_claw = hwMap.get(Servo.class, "left_claw");
        right_claw = hwMap.get(Servo.class, "right_claw");
        wrist.setPosition(WRIST_HOME);
        close_claw();
    }

    public double servo_conv(double degrees){return degrees/180;}
    public void close_claw(){
        left_claw.setPosition(0.2);
        right_claw.setPosition(0.8);
    }
    public void open_claw(){
        left_claw.setPosition(CLAW_HOME);
        right_claw.setPosition(INV_CLAW_HOME);
    }
}
