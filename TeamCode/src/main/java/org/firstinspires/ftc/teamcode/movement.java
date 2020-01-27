package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cColorSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;


@TeleOp(name = "manual")

public class movement extends LinearOpMode
{
    private static final double MAX_POWER = 1;
    private static final int YELLOW_ID   =  8;

    class driveThread extends Thread
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
        driveThread(String tmpNameMotor1,String tmpNameMotor2,String tmpNameMotor3,String tmpNameMotor4)
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
            do
            {
                //fast mode
                if(gamepad1.left_bumper)
                {
                    speed = 0.8;
                }

                //slow mode
                if(gamepad1.right_bumper)
                {
                    speed = 0.3;
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
                /*FL*/motor1Val = -(normelize(sumValues(-gamepad1.left_stick_y,gamepad1.right_stick_x,gamepad1.left_stick_x)))*speed;
                /*FR*/motor2Val = -(normelize(sumValues(gamepad1.left_stick_y,gamepad1.right_stick_x,gamepad1.left_stick_x)))*speed;
                /*BL*/motor3Val = -(normelize(sumValues(-gamepad1.left_stick_y,gamepad1.right_stick_x,-gamepad1.left_stick_x)))*speed;
                /*BR*/motor4Val = (normelize(sumValues(gamepad1.left_stick_y,gamepad1.right_stick_x,-gamepad1.left_stick_x)))*speed;

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
        DcMotor sideMotor;

        String heightMotorName;
        DcMotor heightMotor;

        boolean flag = true;

        liftThread(String tmpSideMotorName ,String tmpHeightMotorName)
        {
            sideMotorName = tmpSideMotorName;
            sideMotor = hardwareMap.dcMotor.get(sideMotorName);
            heightMotorName = tmpHeightMotorName;
            heightMotor = hardwareMap.dcMotor.get(heightMotorName);
        }

        void setFlag(boolean newFlag)
        {
            flag = newFlag;
        }

        public void run()
        {
            while(flag)
            {
                sideMotor.setPower(gamepad1.right_trigger - gamepad1.left_trigger);
                heightMotor.setPower(gamepad1.left_stick_y);
            }
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

        pumpingThread(String tmpLeftPumpName, String tmpRightPumpName,String tmpColorSensorName )
        {
            leftPumpName = tmpLeftPumpName;
            rightPumpName = tmpRightPumpName;
            colorSensorName = tmpColorSensorName;
            leftPump = hardwareMap.dcMotor.get(leftPumpName);
            rightPump = hardwareMap.dcMotor.get(rightPumpName);
            colorSensor =  hardwareMap.get(ModernRoboticsI2cColorSensor.class, colorSensorName);
        }

        public void run()
        {
            colorSensor.enableLed(true);
            while(flag)
            {
                while(gamepad1.a || colorSensor.readUnsignedByte(ModernRoboticsI2cColorSensor.Register.COLOR_NUMBER) == YELLOW_ID)
                {
                    rightPump.setPower(MAX_POWER);
                    leftPump.setPower(-MAX_POWER);
                }
                rightPump.setPower(0);
                leftPump.setPower(0);
            }
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
                if(gamepad1.a)
                {
                    totemCatcher.setPosition(0.5);
                }
                if(gamepad1.b)
                {
                    totemCatcher.setPosition(0);
                }
            }
        }
    }

    @Override
    public void runOpMode ()
    {
        driveThread drive = new driveThread("FL","FR","BL","BR");
        liftThread lift = new liftThread("","");
        catcherThread catcher = new catcherThread("","");
        pumpingThread pump = new pumpingThread("","","");
        lift.start();
        catcher.start();
        pump.start();
        drive.start();
        waitForStart();
        //noinspection StatementWithEmptyBody
        while (opModeIsActive());
        drive.setFlag(false);
        lift.setFlag(false);
        pump.setflag(false);
        catcher.setFlag(false);
    }
}
