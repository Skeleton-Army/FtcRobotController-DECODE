package org.firstinspires.ftc.teamcode.consts;

import com.pedropathing.geometry.Pose;

public class GoalPositions {
    private final static double FIELD_LENGTH = 141.5;

    public final static Pose BLUE_GOAL = new Pose(8.5, FIELD_LENGTH - 9.5);
    public final static Pose RED_GOAL = new Pose(FIELD_LENGTH - BLUE_GOAL.getX(), BLUE_GOAL.getY());

    public final static Pose BLUE_GOAL_FAR = new Pose(3, FIELD_LENGTH);
    public final static Pose RED_GOAL_FAR = new Pose(FIELD_LENGTH - BLUE_GOAL_FAR.getX(), BLUE_GOAL_FAR.getY());

    public final static Pose CLOSE_AUTO_BLUE_GOAL = new Pose(6, FIELD_LENGTH - 2);
    public final static Pose CLOSE_AUTO_RED_GOAL = new Pose(FIELD_LENGTH - CLOSE_AUTO_BLUE_GOAL.getX(), CLOSE_AUTO_BLUE_GOAL.getY());
}