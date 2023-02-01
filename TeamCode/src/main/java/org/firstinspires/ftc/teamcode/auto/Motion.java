package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.util.ElapsedTime;

public abstract class Motion implements Runnable {
    public ElapsedTime runtime = new ElapsedTime();

    public abstract boolean isEnd();
    public abstract void init();
    public abstract void run();
    public abstract void cleanup();
}