package org.firstinspires.ftc.teamcode.Autonomous;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.teamcode.drive.MecanumDriveBase;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;


@Autonomous
//@Disabled
public class Auto_R_basic extends LinearOpMode {
    static final double FEET_PER_METER = 3.28084;
    @Override
    public void runOpMode() throws InterruptedException {

        MecanumDriveBase drive = new MecanumDriveBase(hardwareMap);

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        //move forward 16 in
        Trajectory traj1 = drive.trajectoryBuilder(new Pose2d())
                .forward(21)
                .build();

        //move right while maintaining same heading for 8 in
        Trajectory traj2 = drive.trajectoryBuilder(traj1.end())
                .strafeRight(12)
                .build();

        //move forward 26 in
        Trajectory traj3 = drive.trajectoryBuilder(traj2.end())
                .forward(30)
                .build();

        //move left while maintaining same heading for 8 in
        Trajectory traj4 = drive.trajectoryBuilder(traj3.end())
                .strafeRight(14)
                .build();

        //move back for 50 in
        Trajectory traj5 = drive.trajectoryBuilder(traj4.end())
                .back(40)
                .build();

        //move forward for 54 in
        Trajectory traj6 = drive.trajectoryBuilder(traj5.end())
                .forward(45)
                .build();

        //move left while maintaining same heading for 10 in
        Trajectory traj7 = drive.trajectoryBuilder(traj6.end())
                .strafeRight(13)
                .build();

        //move back for 50 in
        Trajectory traj8 = drive.trajectoryBuilder(traj7.end())
                .back(41)
                .build();

        //Move forward 52
        Trajectory traj9 = drive.trajectoryBuilder(traj8.end())
                .forward(41)
                .build();

        //Move left 7
        Trajectory traj10 = drive.trajectoryBuilder(traj9.end())
                .strafeRight(9)
                .build();

        //move back 45
        Trajectory traj11 = drive.trajectoryBuilder(traj10.end())
                .back(41)
                .build();

        waitForStart();

        drive.setMotorPowers(0.8, 0.8, 0.8, 0.8);

        drive.followTrajectory(traj1);
        drive.followTrajectory(traj2);
        drive.followTrajectory(traj3);
        drive.followTrajectory(traj4);
        drive.followTrajectory(traj5);
        drive.followTrajectory(traj6);
        drive.followTrajectory(traj7);
        drive.followTrajectory(traj8);
        drive.followTrajectory(traj9);
        drive.followTrajectory(traj10);
        drive.followTrajectory(traj11);


        telemetry.clearAll();

        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


    }
}