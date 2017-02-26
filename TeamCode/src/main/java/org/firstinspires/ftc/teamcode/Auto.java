package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * Created by aedanwen on 2017-02-11.
 */

@Autonomous(name = "Auto", group = "Robot Main")
public class Auto extends OpMode {

    long startingTime = 0;
    public final int DRIVING_UNIT = 3666;
    HardwareRobot robot = new HardwareRobot();
    boolean initialWaitComplete = false;

    public void init(){
    }

    public void init_loop(){
        telemetry.addData("Auto.init_loop", null);
    }



    public void loop () {

        if (System.currentTimeMillis() - startingTime > 10000) {
            initialWaitComplete = true;
            telemetry.addData("time until start:", System.currentTimeMillis() - startingTime);
        }

        if (initialWaitComplete) {

            robot.wheels_left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.wheels_right.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            robot.wheels_right.setTargetPosition(DRIVING_UNIT);    //total revolutions of wheel (in steps)
            robot.wheels_left.setTargetPosition(DRIVING_UNIT);     //total revolutions of wheel (in steps)

            robot.wheels_right.setPower(1);
            robot.wheels_left.setPower(1);


            telemetry.addData("left", robot.wheels_left.getCurrentPosition());
            telemetry.addData("right", robot.wheels_right.getCurrentPosition());

            //TODO
            //if run twice, the encoders will not reset (change RunMode to STOP_AND_RESET_ENCODER and then back to RUN_TO_POSITIONM

        }
    }

}
