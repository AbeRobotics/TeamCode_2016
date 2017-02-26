package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * Created by aedanwen on 2017-02-12.
 */
@Autonomous(name = "AutoSandbox Blue", group = "BLUE")
public class AtuoBlueRamp extends OpMode {

    HardwareRobot robot = new HardwareRobot();

    public void init(){
        robot.init(hardwareMap);

    }
    public void start(){

    }
    public void init_loop(){

    }

    public void loop(){

        robot.wheels_left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.wheels_right.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        robot.wheels_right.setTargetPosition(14666);
        robot.wheels_left.setTargetPosition(14666);

        robot.wheels_right.setPower(1);
        robot.wheels_left.setPower(1);

        robot.wheels_right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.wheels_left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

    }
}
