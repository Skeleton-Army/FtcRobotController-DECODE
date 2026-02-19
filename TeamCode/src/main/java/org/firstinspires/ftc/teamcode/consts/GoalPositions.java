package org.firstinspires.ftc.teamcode.consts;

import com.pedropathing.geometry.Pose;

public class GoalPositions {
    private final static double FIELD_LENGTH = 141.5;

    public final static Pose BLUE_GOAL = new Pose(2, FIELD_LENGTH - 2);
    public final static Pose TURRET_BLUE_GOAL = new Pose(2, FIELD_LENGTH - 2);
    public final static Pose RED_GOAL = new Pose(FIELD_LENGTH - BLUE_GOAL.getX(), BLUE_GOAL.getY());
    public final static Pose TURRET_RED_GOAL = new Pose(FIELD_LENGTH - TURRET_BLUE_GOAL.getX(), TURRET_BLUE_GOAL.getY());

    public final static Pose BLUE_GOAL_TOP = new Pose(25, FIELD_LENGTH);
    public final static Pose BLUE_GOAL_BUTTOM = new Pose(0, 119);
    public final static Pose BLUE_GOAL_CORNER = new Pose(0, FIELD_LENGTH);
    public final static Pose RED_GOAL_TOP = new Pose( FIELD_LENGTH - BLUE_GOAL_TOP.getX(), 141.5);
    public final static Pose RED_GOAL_BUTTOM = new Pose(FIELD_LENGTH, 119);
    public final static Pose RED_GOAL_CORNER = new Pose(FIELD_LENGTH, FIELD_LENGTH);


}