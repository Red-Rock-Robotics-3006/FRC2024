package frc.robot.subsystems;

import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;

import java.util.function.IntFunction;

import edu.wpi.first.wpilibj.AddressableLEDBuffer;

public class LEDBuffer extends AddressableLEDBuffer{
    public LEDBuffer(int length)
    {
        super(length);
    }

    public void setRGB(IntFunction<Integer> r, IntFunction<Integer> g, IntFunction<Integer> b)
    {
        this.setRGB(0, this.getLength(), r, g, b);
    }

    public void setRGB(int start, int end, IntFunction<Integer> r, IntFunction<Integer> g, IntFunction<Integer> b)
    {
        int index[] = this.validate(start, end);
        start = index[0];
        end = index[1];
        
        for(int i = start; i < end; i++)
            this.setRGB(i, r.apply(i), g.apply(i), b.apply(i));
    }

    public void setHSV(IntFunction<Integer> h, IntFunction<Integer> s, IntFunction<Integer> v)
    {
        this.setHSV(0, this.getLength(), h, s, v);
    }

    public void setHSV(int start, int end, IntFunction<Integer> h, IntFunction<Integer> s, IntFunction<Integer> v)
    {
        int index[] = this.validate(start, end);
        start = index[0];
        end = index[1];
        
        for(int i = start; i < end; i++)
            this.setHSV(i, h.apply(i), s.apply(i), v.apply(i));
    }

    public void setRGB(int r, int g, int b)
    {
        this.setRGB(0, this.getLength(), r, g, b);
    }

    public void setRGB(int start, int end, int r, int g, int b)
    {
        int index[] = this.validate(start, end);
        start = index[0];
        end = index[1];
        
        for(int i = start; i < end; i++)
            this.setRGB(i, r, g, b);
    }

    public void setHSV(int h, int s, int v)
    {
        this.setRGB(0, this.getLength(), h, s, v);
    }

    public void setHSV(int start, int end, int h, int s, int v)
    {
        int index[] = this.validate(start, end);
        start = index[0];
        end = index[1];
        
        for(int i = start; i < end; i++)
            this.setHSV(i, h, s, v);
    }

    public void setLED(Color color)
    {
        this.setLED(0, this.getLength(), color);
    }

    public void setLED(int start, int end, Color color)
    {
        int index[] = this.validate(start, end);
        start = index[0];
        end = index[1];

        for(int i = start; i < end; i++)
            this.setLED(i, color);
    }

    public void setLED(Color8Bit color)
    {
        this.setLED(0, this.getLength(), color);
    }

    public void setLED(int start, int end, Color8Bit color)
    {
        int index[] = this.validate(start, end);
        start = index[0];
        end = index[1];

        for(int i = start; i < end; i++)
            this.setLED(i, color);
    }


    public void test()
    {
        
    }

    private int[] validate(int start, int end)
    {
        if(end < start)
        {
            int e = end;
            end = start;
            start = e;
        }
        if(end > this.getLength())
        {
            System.out.println("ERRORtest: " + end);
            // throw new IndexOutOfBoundsException(end);
        }
        if(start < 0)
        {
            System.out.println("ERRORteat: " + start);
            // throw new IndexOutOfBoundsException(start);
        }
        
        return new int[]{start, end};
    }
}
