package org.firstinspires.ftc.teamcode.Autonomous;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.MecanumDriveBase;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;


@Autonomous
//@Disabled
public class Auto_L_basic_park_arm extends LinearOpMode {
    static final double FEET_PER_METER = 3.28084;

    private DcMotor liftRotator = null;
    private DcMotor liftExtender = null;

    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {

        MecanumDriveBase drive = new MecanumDriveBase(hardwareMap);

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        liftRotator = hardwareMap.get(DcMotor.class, "lift_rotator");
        liftExtender = hardwareMap.get(DcMotor.class, "lift_extender");

        liftRotator.setDirection(DcMotor.Direction.FORWARD);
        liftExtender.setDirection(DcMotor.Direction.REVERSE);

        liftRotator.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        liftExtender.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //move forward 47 in
        Trajectory traj3 = drive.trajectoryBuilder(new Pose2d())
                .forward(47)
                .build();

//        //move right while maintaining same heading for 8 in
//        Trajectory traj2 = drive.trajectoryBuilder(traj1.end())
//                .strafeLeft(8)
//                .build();
//
//        //move forward 26 in
//        Trajectory traj3 = drive.trajectoryBuilder(traj2.end())
//                .forward(26)
//                .build();

        //move left while maintaining same heading for 13 in
        Trajectory traj4 = drive.trajectoryBuilder(traj3.end())
                .strafeLeft(11)
                .build();

        //move back for 50 in
        Trajectory traj5 = drive.trajectoryBuilder(traj4.end())
                .back(50)
                .build();

        //move forward for 46 in
        Trajectory traj6 = drive.trajectoryBuilder(traj5.end())
                .forward(46)
                .build();

        //move left while maintaining same heading for 12 in
        Trajectory traj7 = drive.trajectoryBuilder(traj6.end())
                .strafeLeft(11)
                .build();

        //move back for 43 in
        Trajectory traj8 = drive.trajectoryBuilder(traj7.end())
                .back(43)
                .build();

        //Move forward 42
        Trajectory traj9 = drive.trajectoryBuilder(traj8.end())
                .forward(42)
                .build();

        //Move left 9
        Trajectory traj10 = drive.trajectoryBuilder(traj9.end())
                .strafeLeft(11)
                .build();

        //move back 39
        Trajectory traj11 = drive.trajectoryBuilder(traj10.end())
                .back(39)
                .build();

        //move forward 40
        Trajectory traj12 = drive.trajectoryBuilder(traj11.end())
                .forward(40)
                .build();

        // do cool stuff
        Trajectory traj13 = drive.trajectoryBuilder(traj12.end().plus(
                        new Pose2d(0, 0, Math.toRadians(285))), false)
                .forward(25)
                .build();

        Trajectory traj14 = drive.trajectoryBuilder(traj13.end())
                .forward(15)
                .addTemporalMarker(0, () -> {
                    runtime.reset();
                    while (runtime.seconds() < 2) {
                        liftRotator.setPower(0.5);
                    }
                })
                .addTemporalMarker(0.5, () -> {
                    runtime.reset();
                    while (runtime.seconds() < 1) {
                        liftExtender.setPower(0.5);
                    }
                })
                .addTemporalMarker(1.5, () -> {
                    runtime.reset();
                    while (runtime.seconds() < 0.25) {
                        liftRotator.setPower(-0.05);
                    }
                })
                .build();

        waitForStart();

        drive.setMotorPowers(0.8, 0.8, 0.8, 0.8);

//        drive.followTrajectory(traj1);
//        drive.followTrajectory(traj2);
        drive.followTrajectory(traj3);
        drive.followTrajectory(traj4);
        drive.followTrajectory(traj5);
        drive.followTrajectory(traj6);
        drive.followTrajectory(traj7);
        drive.followTrajectory(traj8);
        drive.followTrajectory(traj9);
        drive.followTrajectory(traj10);
        drive.followTrajectory(traj11);

//        drive.turn(Math.toRadians(90));
//
        drive.followTrajectory(traj12);
        drive.followTrajectory(traj13);

        drive.setMotorPowers(0.08, 0.08, 0.08, 0.08);

        drive.followTrajectory(traj14);


        telemetry.clearAll();

        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


    }
}