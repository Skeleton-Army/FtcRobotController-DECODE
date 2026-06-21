package org.firstinspires.ftc.teamcode.consts;

import com.pedropathing.geometry.Pose;

public class GoalPositions {
    private final static double FIELD_LENGTH = 141.5;

    public final static Pose BLUE_GOAL = new Pose(8.5, FIELD_LENGTH - 9.5);
    public final static Pose RED_GOAL = new Pose(FIELD_LENGTH - BLUE_GOAL.getX(), BLUE_GOAL.getY());

    public final static Pose BLUE_GOAL_FAR = new Pose(8.5, FIELD_LENGTH - 9.5);
    public final static Pose RED_GOAL_FAR = new Pose(FIELD_LENGTH - BLUE_GOAL_FAR.getX(), BLUE_GOAL_FAR.getY());
}