package org.firstinspires.ftc.teamcode.consts;

import com.pedropathing.geometry.Pose;

public class GoalPositions {
    public final static Pose BLUE_GOAL = new Pose(2, 142);
    public final static Pose RED_GOAL = new Pose(144 - BLUE_GOAL.getX(), BLUE_GOAL.getY());
}