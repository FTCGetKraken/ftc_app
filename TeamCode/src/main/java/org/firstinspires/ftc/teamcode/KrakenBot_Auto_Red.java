package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.*;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/**
 * Created by eriche on 10/28/17.
 */
@Autonomous(name="KrakenBot:AutoRed",group="final")
public class KrakenBot_Auto_Red extends LinearOpMode {
    public KrakenBot robot;
    public ElapsedTime runtime = new ElapsedTime();
    
    public void runOpMode() {
        robot = new KrakenBot(new Orientation(true,true));
        robot.init(hardwareMap);
        waitForStart();
        robot.autoWithJewels();
    }
}
