package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cColorSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import java.util.concurrent.TimeUnit;


@TeleOp(name = "manual")

public class movement extends LinearOpMode
{
    private static final double MAX_POWER = 1;
    private static final int YELLOW_ID   =  8;
    private static final double KP = -1.0 / 200;


    private static final double  DC_SERVO_MAX_POWER  = 0.75;

    private class DcServo extends Thread
    {
        String name;
        private DcMotor encoded_motor;
        private double target = 0;
        private double offset;
        boolean flag = true;
        private double position = 0;
        double privateKP;
        DcServo(String tmpName, double tmpPrivateKp)
        {
            name = tmpName;
            encoded_motor = hardwareMap.dcMotor.get(name);
            offset = encoded_motor.getCurrentPosition();
            privateKP = tmpPrivateKp;
        }

        private void setTarget(double newVal)
        {
            target = newVal;
        }

        void setFlag(boolean newState)
        {
            flag = false;
        }

        @Override
        public void run()
        {
            while (flag)
            {
                try
                {
                    TimeUnit.MILLISECONDS.sleep(10);
                } catch (InterruptedException e)
                {
                    e.printStackTrace();
                }
                position = encoded_motor.getCurrentPosition() - offset;
                double error = target - position;
                double power = error * KP * privateKP;
                if (power > DC_SERVO_MAX_POWER) power = DC_SERVO_MAX_POWER;
                if (power < -DC_SERVO_MAX_POWER) power = -DC_SERVO_MAX_POWER;
                encoded_motor.setPower(-power);
            }
        }
    }



    class driveThreadV2 extends Thread
    {
        String motor1Name;
        String motor2Name;
        String motor3Name;
        String motor4Name;
        double motor1Val;
        double motor2Val;
        double motor3Val;
        double motor4Val;
        DcMotor motor1;
        DcMotor motor2;
        DcMotor motor3;
        DcMotor motor4;
        boolean flag = true;
        driveThreadV2(String tmpNameMotor1,String tmpNameMotor2,String tmpNameMotor3,String tmpNameMotor4)
        {
            motor1Name = tmpNameMotor1;
            motor2Name = tmpNameMotor2;
            motor3Name = tmpNameMotor3;
            motor4Name = tmpNameMotor4;

            motor1 = hardwareMap.dcMotor.get(motor1Name);
            motor2 = hardwareMap.dcMotor.get(motor2Name);
            motor3 = hardwareMap.dcMotor.get(motor3Name);
            motor4 = hardwareMap.dcMotor.get(motor4Name);
        }

        void setFlag(boolean newFlag)
        {
            flag = newFlag;
        }

        Double normelize(Double value)
        {
            double maxSpeed = 1;
            if(value > maxSpeed) value = maxSpeed;
            else if (value < -maxSpeed) value = -maxSpeed;
            return value;
        }

        Double sumValues(double val1, double val2 , double val3)
        {
            return val1+val2+val3;
        }


        public void run()
        {
            double speed = 1;
            double side_factor = 0.6;
            do
            {
                //fast mode
                if(gamepad1.left_bumper)
                {
                    speed = 1;
                    side_factor = 0.6;
                }

                //slow mode
                if(gamepad1.right_bumper)
                {
                    speed = 0.3;
                    side_factor = 1.6;
                }

                //break mode
                if(gamepad1.a)
                {
                    motor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                    motor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                    motor3.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                    motor4.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                }
                //float mode
                else
                {
                    motor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                    motor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                    motor3.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                    motor4.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                }
                //movement function
                /*FL*/motor1Val = -(normelize(sumValues( gamepad1.left_stick_y, gamepad1.right_stick_x,-gamepad1.left_stick_x * side_factor)))*speed;
                /*FR*/motor2Val = -(normelize(sumValues(-gamepad1.left_stick_y, gamepad1.right_stick_x,-gamepad1.left_stick_x * side_factor)))*speed;
                /*BL*/motor3Val = -(normelize(sumValues( gamepad1.left_stick_y, gamepad1.right_stick_x, gamepad1.left_stick_x * side_factor)))*speed;
                /*BR*/motor4Val = -(normelize(sumValues(-gamepad1.left_stick_y, gamepad1.right_stick_x, gamepad1.left_stick_x * side_factor)))*speed;

                if(motor1Val==-1 && motor3Val == 1)
                {
                    motor2Val = 1;
                    motor4Val = -1;
                }


                if(motor1Val==-1 && motor3Val == 1)
                {
                    motor2Val = -1;
                    motor4Val = -1;
                }

                else if(motor1Val == 1 && motor3Val == -1)
                {
                    motor2Val = 1;
                    motor4Val = 1;
                }
                motor1.setPower(motor1Val);
                motor2.setPower(motor2Val);
                motor3.setPower(motor3Val);
                motor4.setPower(motor4Val);
            }while (flag);

        }
    }

    class liftThread extends Thread
    {
        String sideMotorName;
        DcServo sideMotor;

        String heightMotorName;
        DcServo heightMotor;

        boolean flag = true;

        liftThread(String tmpSideMotorName ,String tmpHeightMotorName)
        {
            sideMotorName = tmpSideMotorName;
            sideMotor = new DcServo("side", 1);
            heightMotorName = tmpHeightMotorName;
            heightMotor = new DcServo(heightMotorName,1);
        }

        void setFlag(boolean newFlag)
        {
            flag = newFlag;
        }

        public void run()
        {

            double side_position = 0;
            double height_position = 0;

            double pTime = System.currentTimeMillis() / 1000.0;

            sideMotor.start();
            heightMotor.start();
            while(flag)
            {
                double currentTime = System.currentTimeMillis() / 1000.0;
                double dt  = currentTime - pTime;
                pTime = currentTime;
                double sideMotorPower = gamepad2.right_trigger - gamepad2.left_trigger;
                //if(Math.abs(sideMotorPower) < 0.05) sideMotorPower = 0;
                side_position += sideMotorPower * dt * 750;
                if(side_position < 0) side_position = 0;
                if(side_position > 1475) side_position = 1475;
                height_position += gamepad2.left_stick_y * dt * 1000;
                if(height_position > 0) height_position = 0;
                if(height_position < -1900) height_position = -1900;
                telemetry.addData("height target", height_position);
                telemetry.addData("height motor enc: ", heightMotor.position);
                telemetry.update();
                sideMotor.setTarget(side_position);
                heightMotor.setTarget(height_position);
            }
            sideMotor.setFlag(false);
            heightMotor.setFlag(false);

        }
    }

    class pumpingThread extends Thread
    {
        String leftPumpName;
        String rightPumpName;
        DcMotor leftPump;
        DcMotor rightPump;
        ModernRoboticsI2cColorSensor colorSensor;
        String colorSensorName;
        boolean flag = true;

        void  setflag(boolean newFlag)
        {
            flag = newFlag;
        }

        pumpingThread(String tmpLeftPumpName, String tmpRightPumpName)
        {
            leftPumpName = tmpLeftPumpName;
            rightPumpName = tmpRightPumpName;
            //colorSensorName = tmpColorSensorName;
            leftPump = hardwareMap.dcMotor.get(leftPumpName);
            rightPump = hardwareMap.dcMotor.get(rightPumpName);
            //colorSensor =  hardwareMap.get(ModernRoboticsI2cColorSensor.class, colorSensorName);
        }

        public void run()
        {
            //colorSensor.enableLed(true);
            int state = 0;
            while(flag)
            {
                switch(state)
                {
                    case 0: //stop
                        rightPump.setPower(0);
                        leftPump.setPower(0);
                        break;
                    case 1: //in
                        rightPump.setPower(MAX_POWER);
                        leftPump.setPower(-MAX_POWER * 0.7);
                        break;
                    case 2: //out
                        rightPump.setPower(-MAX_POWER);
                        leftPump.setPower(MAX_POWER);
                        break;
                }
                    if(gamepad2.dpad_down)
                    {
                        state = 2;
                    }

                    if(gamepad2.dpad_up)
                    {
                        state = 1;
                    }
                    if(gamepad2.dpad_right || gamepad2.dpad_left)
                    {
                        state = 0;
                    }
                }
                rightPump.setPower(0);
                leftPump.setPower(0);
            }
        }


    class catcherThread extends Thread
    {
        String totemCatcherName;
        String blockCatcherName;

        Servo totemCatcher;
        Servo blockCatcher ;

        boolean flag = true;

        catcherThread(String tmpTotemCatcherName , String tmpBlockCatcherName)
        {
            totemCatcherName = tmpTotemCatcherName;
            blockCatcherName = tmpBlockCatcherName;

            totemCatcher = hardwareMap.get(Servo.class,totemCatcherName);
            blockCatcher = hardwareMap.get(Servo.class,blockCatcherName);
        }

        void setFlag(boolean newFlag)
        {
            flag = newFlag;
        }

        public void run()
        {
            while(flag)
            {
                if(gamepad2.right_bumper)
                {
                    blockCatcher.setPosition(0.7);
                }
                if(gamepad2.left_bumper)
                {
                    blockCatcher.setPosition(0);
                }

                if(gamepad2.x)
                {
                    totemCatcher.setPosition(0.6);
                }
                if(gamepad2.y)
                {
                    totemCatcher.setPosition(0.4);
                }
            }
        }
    }


    class foundation_move extends Thread
    {
        Servo move;

        String move_name;

        boolean flag = true;

        foundation_move(String tmp_move_name)
        {
            move_name = tmp_move_name;
            move = hardwareMap.get(Servo.class,move_name);
        }

        void setFlag(boolean new_flag)
        {
            flag = new_flag;
        }

        public void run()
        {
            move.setPosition(0);
            while (flag)
            {
                if(gamepad2.a)
                {
                    move.setPosition(0.6);
                }
                if(gamepad2.b)
                {
                    move.setPosition(0.2);
                }
            }
        }

    }


    @Override
    public void runOpMode ()
    {
        driveThreadV2 driveV2 = new driveThreadV2("FL","FR","BL","BR");
        liftThread lift = new liftThread("side","height");
        catcherThread catcher = new catcherThread("totem","block");
        foundation_move move = new foundation_move("move");
        pumpingThread pump = new pumpingThread("left_pump","right_pump");
        driveV2.start();
        move.start();
        lift.start();
        catcher.start();
        pump.start();
        waitForStart();
        //noinspection StatementWithEmptyBody
        while (opModeIsActive());
        driveV2.setFlag(false);
        lift.setFlag(false);
        pump.setflag(false);
        move.setFlag(false);
        catcher.setFlag(false);
    }
}
