package frc.robot.subsystems;
import java.util.function.IntFunction;

import edu.wpi.first.wpilibj.AddressableLEDBuffer;

public class Buffer extends AddressableLEDBuffer{
    int[] h;

    public Buffer(int length)
    {
        super(length);
        this.h = new int[length];
    }

    @Override
    public void setHSV(int index, int h, int s, int v)
    {
        super.setHSV(index, h, s, v);
        this.h[index] = h;
    }

    public void setHSV(int start, int end, IntFunction<Integer> h, int s, int v)
    {
        for(int i = start; i < end; i++)
            this.setHSV(i, h.apply(i), s, v);
    }
}
