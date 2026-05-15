package org.firstinspires.ftc.teamcode.utilities;

public class Pair<T1, T2> {
    private final T1 first;
    private final T2 second;

    public Pair(T1 first, T2 second) {
        this.first = first;
        this.second = second;
    }

    public T1 getRight() {
        return first;
    }

    public T2 getLeft() {
        return second;
    }
}
