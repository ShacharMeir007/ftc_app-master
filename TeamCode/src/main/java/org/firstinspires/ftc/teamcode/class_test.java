package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name="test")
@Disabled

public class class_test extends LinearOpMode {
    @Override
    public void runOpMode()
    {
        DcMotor Tmotor =  hardwareMap.dcMotor.get("encoded");
        telemetry.addData("start","start");
        DcToServo test = new DcToServo("test",Tmotor,100,gamepad1,hardwareMap,telemetry);
        telemetry.addData("end","end");
        test.DrivePosition();
        telemetry.update();



    }
}
