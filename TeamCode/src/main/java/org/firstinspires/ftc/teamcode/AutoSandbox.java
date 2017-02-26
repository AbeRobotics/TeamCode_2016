package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * Created by aedanwen on 2017-02-11.
 */

@Autonomous(name = "AutoSandbox", group = "Robot Main")
public class AutoSandbox extends OpMode {

    long startingTime = 0;
    public final int DRIVING_UNIT = 3666;

    HardwareRobot robot = new HardwareRobot();
    boolean initialWaitComplete = false;
    boolean readyForDrive = false;

    public void start(){
        startingTime = System.currentTimeMillis();

    }

    public void init(){
        robot.init(hardwareMap);
        telemetry.addData("AutoSandbox.init", null);
        robot.wheels_right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.wheels_left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

    }

    public void init_loop(){
        telemetry.addData("AutoSandbox.init_loop", null);
    }

    public void stop(){
        telemetry.addData("AutoSandbox.stop", null);
        robot.wheels_left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.wheels_right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        robot.wheels_left.setPower(0);
        robot.wheels_right.setPower(0);
    }

    public void loop () {

        if (System.currentTimeMillis() - startingTime > 10000) {
            initialWaitComplete = true;
            telemetry.addData("time until start:", System.currentTimeMillis() - startingTime);
        }

        if (initialWaitComplete && !readyForDrive) {

            //change settings for driving
            readyForDrive = true;
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

            if (Math.abs(robot.wheels_right.getCurrentPosition()) >= DRIVING_UNIT) {
                this.stop();

            }

            //TODO
            //if run twice, the encoders will not reset (change RunMode to STOP_AND_RESET_ENCODER and then back to RUN_TO_POSITIONM

        }
    }


    private void writeToTelemetry(String message) {
        telemetry.addData("***" + message + "@ time" , System.currentTimeMillis() - startingTime);
    }

}
