package org.firstinspires.ftc.teamcode.consts;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.geometry.Pose;

@Config
public class GoalPositions {
    public final static double FIELD_LENGTH = 189.5;
    public final static double HALF_FIELD_LENGTH = 189.5 / 2;

    public final static Pose BLUE_GOAL = new Pose(10, FIELD_LENGTH - 5);
    public final static Pose RED_GOAL = new Pose(FIELD_LENGTH - BLUE_GOAL.getX(), BLUE_GOAL.getY());

    public final static Pose RED_GOAL_GEMS = new Pose (HALF_FIELD_LENGTH + BLUE_GOAL.getX(), BLUE_GOAL.getY());
    public final static Pose BLUE_GOAL_GEMS = new Pose(HALF_FIELD_LENGTH - BLUE_GOAL.getX(), BLUE_GOAL.getY());

    private final static double GOOFY_OFFSET_X = 10; // For some mysterious splendid wonderful reason this is needed because it shoots way left and i have NO clue why
    private final static double FAR_OFFSET_X = 5;
    public final static Pose BLUE_GOAL_FAR = new Pose(GOOFY_OFFSET_X + FAR_OFFSET_X, FIELD_LENGTH);
    public final static Pose RED_GOAL_FAR = new Pose(FIELD_LENGTH + FAR_OFFSET_X, BLUE_GOAL_FAR.getY());
}