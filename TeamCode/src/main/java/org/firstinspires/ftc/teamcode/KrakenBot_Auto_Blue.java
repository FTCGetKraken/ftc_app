package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Created by eriche on 10/28/17.
 */
@Autonomous(name="KrakenBot:AutoBlue",group="final")
public class KrakenBot_Auto_Blue extends LinearOpMode {
    public KrakenBot robot;
    public ElapsedTime runtime = new ElapsedTime();
    
    public void runOpMode() {
        robot = new KrakenBot(new KrakenOrientation(true,false));
        robot.init(hardwareMap);
        waitForStart();
        robot.autoWithJewels();
    }
}
