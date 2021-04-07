package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

public abstract class CommonOpMode extends LinearOpMode {
    DcMotor FRmotor;
    DcMotor RRmotor;
    DcMotor FLmotor;
    DcMotor RLmotor;
    DcMotor Flywheel;
    DcMotor Worm;
    Servo Grabber;
    Servo Pushrod;
    DcMotor Collector;
    DcMotorEx FlywheelEx;
    OpenCvCamera webcam;
    BingusPipeline pipeline;
    Boolean ExecuteFlag;
    BNO055IMU imu;
    Orientation angles = new Orientation();
    Acceleration gravity = new Acceleration();
    boolean isFlywheelRunning = false;
    public BingusPipeline.RandomizationFactor ringData;
    public ElapsedTime whenAreWe = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
    final int LogPower=3;
    boolean grab = false,push = false,collector = false,flywheel = false,flick = false;
    boolean prevgrab,prevpush,prevfly,prevcoll,prevflick;
    double for_axis,strafe_axis,turn_axis,worm_axis;
    public void Initialize(HardwareMap hardwareMap,boolean isAuto) {
        FRmotor = hardwareMap.get(DcMotor.class, "FRmotor");
        RRmotor = hardwareMap.get(DcMotor.class, "RRmotor");
        FLmotor = hardwareMap.get(DcMotor.class, "FLmotor");
        RLmotor = hardwareMap.get(DcMotor.class, "RLmotor");
        Flywheel = hardwareMap.get(DcMotor.class, "FWmotor");
        Worm = hardwareMap.get(DcMotor.class, "Wmotor");
        Grabber = hardwareMap.get(Servo.class, "Gservo");
        Pushrod = hardwareMap.get(Servo.class, "Pservo");
        Collector = hardwareMap.get(DcMotor.class, "Cmotor");
        FRmotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FLmotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RRmotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RLmotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Flywheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        Worm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FRmotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RRmotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FLmotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RLmotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Flywheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FRmotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RRmotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        FLmotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RLmotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Flywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        FLmotor.setDirection(DcMotorSimple.Direction.REVERSE);
        RLmotor.setDirection(DcMotorSimple.Direction.REVERSE);
        Grabber.scaleRange(0.16,0.66);
        Pushrod.scaleRange(0.19,0.3);
        FlywheelEx = (DcMotorEx)(Flywheel);
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "calib.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
        if(isAuto) {
            int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
            webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
            pipeline = new BingusPipeline();
            webcam.setPipeline(pipeline);
            webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
                @Override
                public void onOpened() {
                    webcam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
                }
            });
            telemetry.addLine("Waiting for start.");
            telemetry.addLine("Please calibrate starting position.");
            telemetry.update();
        }
    }
    public void ReadyPeripherals(){
        Grabber.setPosition(0);
        Pushrod.setPosition(0);
        whenAreWe.reset();
        ExecuteFlag=false;
    }
    public void AutoRingLaunch(){
        MoveWithEncoder(1890, 2);
        TurnBySeconds(90,1);
        LaunchSeveralRings(3);
        TurnBySeconds(90,0);
        if(ringData!=BingusPipeline.RandomizationFactor.ZERO) {
            MoveWithEncoder(550, 3);
            Collector.setPower(0.75);
            MoveWithEncoder(1870,0);
            Collector.setPower(-0.75);
            sleep(100);
            MoveWithEncoder(1860,2);
            MoveWithEncoder(550,1);
            TurnBySeconds(90,1);
            if(ringData==BingusPipeline.RandomizationFactor.ONE)LaunchSeveralRings(1);
            else LaunchSeveralRings(3);
            TurnBySeconds(90,0);
        }
        TurnBySeconds(1450,1);
    }
    public void MoveByMillimetres(float millis, int direction){
        //direction counted from 0, being backwards, counterclockwise
        //0=backward, 1=right, 2=forward, 3=left
        ElapsedTime localTime = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        while (localTime.time() <= millis * 1.135) { //what the fuck am i doing
            RLmotor.setPower(Math.signum((direction - 1) * 2 - 1));
            RRmotor.setPower(Math.signum(direction % 3 * 2 - 1));
            FLmotor.setPower(Math.signum(direction % 3 * 2 - 1));
            FRmotor.setPower(Math.signum((direction - 1) * 2 - 1));
        }
        RLmotor.setPower(0);
        RRmotor.setPower(0);
        FLmotor.setPower(0);
        FRmotor.setPower(0);
        sleep(250);
    }
    public void MoveWithEncoder(int millis,int direction){
        //direction counted from 0, being backwards, counterclockwise
        //0=backward, 1=right, 2=forward, 3=left
        FRmotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FRmotor.setTargetPosition((int)(millis*((direction-1)*2-1)*1.71));
        FRmotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        FRmotor.setPower(1);
        while(FRmotor.getCurrentPosition()!=FRmotor.getTargetPosition()&&opModeIsActive()){
            RLmotor.setPower(Math.signum((direction-1)*2-1));
            RRmotor.setPower(Math.signum(direction%3*2-1));
            FLmotor.setPower(Math.signum(direction%3*2-1));
            idle();
        }
        RLmotor.setPower(0);
        RRmotor.setPower(0);
        FLmotor.setPower(0);
        FRmotor.setPower(0);
        sleep(250);
    }
    public void DeployArm(){
        Worm.setPower(-1);
        sleep(2100);
        Worm.setPower(0);
        Grabber.setPosition(1);
        sleep(2000);
    }
    public void RetractArm(){
        Worm.setPower(1);
        sleep(2099);
        Worm.setPower(0);
    }
    public void LaunchSeveralRings(int amount) {
        ElapsedTime localTime = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        Flywheel.setPower(1);
        while (localTime.time() <= 3000 && opModeIsActive()) {
        }
        Pushrod.setPosition(1);
        while (localTime.time() <= 3100 && opModeIsActive()) {
        }
        Pushrod.setPosition(0);
        for (int i = 0; i <= amount--; i++) {
            localTime.reset();
            while (localTime.time() <= 2000 && opModeIsActive()) {
            }
            Pushrod.setPosition(1);
            while (localTime.time() <= 2100 && opModeIsActive()) {
            }
            Pushrod.setPosition(0);
        }
        Flywheel.setPower(0);
    }
    public void TurnBySeconds(int millis, int direction){
        ElapsedTime localTime = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        while (localTime.time() <= millis && opModeIsActive()) { //what the fuck am i doing
            RLmotor.setPower(1 - direction * 2);
            RRmotor.setPower(direction * 2 - 1);
            FLmotor.setPower(1 - direction * 2);
            FRmotor.setPower(direction * 2 - 1);
        }
        RLmotor.setPower(0);
        RRmotor.setPower(0);
        FLmotor.setPower(0);
        FRmotor.setPower(0);
        sleep(100);
    }
    public static double logarithmifyInput(double input, int power) {
        return Math.abs(Math.pow(input,power))*Math.signum(input);
    }
    public void composeInputs(){
        prevgrab=grab;
        prevpush=push;
        prevfly=flywheel;
        prevcoll=collector;
        prevflick=flick;
        for_axis = logarithmifyInput(gamepad1.left_stick_y,LogPower);
        strafe_axis = logarithmifyInput(gamepad1.left_stick_x,LogPower);
        turn_axis = logarithmifyInput(gamepad1.right_stick_x,LogPower);
        worm_axis = logarithmifyInput(gamepad2.left_stick_y,LogPower);
        flick = gamepad1.x;
        grab = gamepad2.left_bumper;
        flywheel = gamepad2.right_bumper;
        push = ((int)(gamepad2.right_trigger+0.25) != 0);
        collector = (gamepad2.dpad_down)||(gamepad2.dpad_up);
    }
    public void operatePeripherals(){
        FRmotor.setPower(for_axis - strafe_axis + turn_axis);
        RRmotor.setPower(for_axis + strafe_axis + turn_axis);
        FLmotor.setPower(for_axis + strafe_axis - turn_axis);
        RLmotor.setPower(for_axis - strafe_axis - turn_axis);
        Worm.setPower(worm_axis);
        if(!prevgrab && grab) {
            Grabber.setPosition(1-Grabber.getPosition());
        }
        if(!prevpush && push){
            Pushrod.setPosition(1);
            sleep(100);
            Pushrod.setPosition(0);
        }
        if(!prevfly&&flywheel){
            FlywheelEx.setVelocity((4000-(isFlywheelRunning?1:0)*4000)*28.0/60.0);
            isFlywheelRunning=!isFlywheelRunning;
        }
        if(!prevcoll&&collector){
            if(gamepad2.dpad_down){
                Collector.setPower(-0.75-Math.abs(Collector.getPower())*Math.signum(Math.signum(Collector.getPower())-1));
            }
            else Collector.setPower(0.75-Collector.getPower()*Math.signum(Math.signum(Collector.getPower())+1));
        }
        if(!prevflick&&flick)OrientToDegrees(-10);
    }
    public void OrientToDegrees(float angle){
        float currentAngle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
        float err = (currentAngle-angle)/180;
        float prev_err=0, integral=0, derivative=err;
        while(((int)currentAngle<(int)angle||(int)currentAngle>(int)angle)&&opModeIsActive()||derivative>0.2){
            currentAngle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
            err = (currentAngle-angle)/180;
            float proportional = err;
            integral = integral+err;
            derivative = err-prev_err;
            float output = 2*proportional+0.03f*integral+1*derivative;
            if(prev_err*err<0)integral=0;
            prev_err=err;
            FLmotor.setPower(-output*0.2);
            RLmotor.setPower(-output*0.2);
            FRmotor.setPower(output*0.2);
            RRmotor.setPower(output*0.2);
            telemetry.addData("Heading in degrees:",currentAngle);
            telemetry.update();
        }
        FLmotor.setPower(0);
        FRmotor.setPower(0);
        RRmotor.setPower(0);
        RLmotor.setPower(0);
        sleep(50);
        currentAngle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
        if((int)currentAngle!=(int)angle)OrientToDegrees(angle);
    }
}