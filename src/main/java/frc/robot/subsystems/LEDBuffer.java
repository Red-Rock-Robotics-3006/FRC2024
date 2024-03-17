package frc.robot.subsystems;

import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;

import java.util.function.IntFunction;

import edu.wpi.first.wpilibj.AddressableLEDBuffer;

public class LEDBuffer extends AddressableLEDBuffer{

    int[] r, g, b;
    int[] h, s, v;

    public LEDBuffer(int length)
    {
        super(length);
        this.r = new int[length];
        this.g = new int[length];
        this.b = new int[length];
        this.h = new int[length];
        this.s = new int[length];
        this.v = new int[length];
    }

    @Override
    public void setRGB(int index, int r, int g, int b)
    {
        super.setRGB(index, r, g, b);
        this.r[index] = r;
        this.g[index] = g;
        this.b[index] = b;
    }

    @Override
    public void setHSV(int index, int h, int s, int v)
    {
        super.setHSV(index, h, s, v);
        this.h[index] = h;
        this.s[index] = s;
        this.v[index] = v;
    }

    public int getH(int index)
    {
        return this.h[index];
    }

    /**
     * Sets all leds in the buffer using a generator.
     *
     * @param r the generator for the r value, based on led index, results in [0-255]
     * @param g the generator for the g value, based on led index, results in [0-255]
     * @param b the generator for the b value, based on led index, results in [0-255]
     */
    public void setRGB(IntFunction<Integer> r, IntFunction<Integer> g, IntFunction<Integer> b)
    {
        this.setRGB(0, this.getLength(), r, g, b);
    }

    /**
     * Sets a range of leds in the buffer using a generator.
     *
     * @param start the index at the start of the range
     * @param end the index at the end of the range
     * @param r the generator for the r value, based on led index, results in [0-255]
     * @param g the generator for the g value, based on led index, results in [0-255]
     * @param b the generator for the b value, based on led index, results in [0-255]
     */
    public void setRGB(int start, int end, IntFunction<Integer> r, IntFunction<Integer> g, IntFunction<Integer> b)
    {
        int index[] = this.validate(start, end);
        start = index[0];
        end = index[1];
        
        for(int i = start; i < end; i++)
            this.setRGB(i, r.apply(i), g.apply(i), b.apply(i));
    }

    /**
     * Sets all leds in the buffer using a generator.
     *
     * @param h the generator for the h value, based on led index, results in [0-180)
     * @param s the generator for the s value, based on led index, results in [0-255]
     * @param v the generator for the v value, based on led index, results in [0-255]
     */
    public void setHSV(IntFunction<Integer> h, IntFunction<Integer> s, IntFunction<Integer> v)
    {
        this.setHSV(0, this.getLength(), h, s, v);
    }

    /**
     * Sets a range of leds in the buffer using a generator.
     *
     * @param start the index at the start of the range
     * @param end the index at the end of the range
     * @param h the generator for the h value, based on led index, results in [0-180)
     * @param s the generator for the s value, based on led index, results in [0-255]
     * @param v the generator for the v value, based on led index, results in [0-255]
     */
    public void setHSV(int start, int end, IntFunction<Integer> h, IntFunction<Integer> s, IntFunction<Integer> v)
    {
        int index[] = this.validate(start, end);
        start = index[0];
        end = index[1];
        
        for(int i = start; i < end; i++)
            this.setHSV(i, h.apply(i), s.apply(i), v.apply(i));
    }

    /**
     * Sets all leds in the buffer.
     *
     * @param r the r value [0-255]
     * @param g the g value [0-255]
     * @param b the b value [0-255]
     */
    public void setRGB(int r, int g, int b)
    {
        this.setRGB(0, this.getLength(), r, g, b);
    }

    /**
     * Sets a range of leds in the buffer.
     *
     * @param start the index at the start of the range
     * @param end the index at the end of the range
     * @param r the r value [0-255]
     * @param g the g value [0-255]
     * @param b the b value [0-255]
     */
    public void setRGB(int start, int end, int r, int g, int b)
    {
        int index[] = this.validate(start, end);
        start = index[0];
        end = index[1];
        
        for(int i = start; i < end; i++)
            this.setRGB(i, r, g, b);
    }

    /**
     * Sets all leds in the buffer.
     *
     * @param h the h value [0-180)
     * @param s the s value [0-255]
     * @param v the v value [0-255]
     */
    public void setHSV(int h, int s, int v)
    {
        this.setHSV(0, this.getLength(), h, s, v);
    }

    /**
     * Sets a range of leds in the buffer.
     *
     * @param start the index at the start of the range
     * @param end the index at the end of the range
     * @param h the h value [0-180)
     * @param s the s value [0-255]
     * @param v the v value [0-255]
     */
    public void setHSV(int start, int end, int h, int s, int v)
    {
        int index[] = this.validate(start, end);
        start = index[0];
        end = index[1];
        
        for(int i = start; i < end; i++)
            this.setHSV(i, h, s, v);
    }

    /**
     * Sets all LEDs in the buffer.
     *
     * @param color The color of the LEDs
     */
    public void setLED(Color color)
    {
        this.setLED(0, this.getLength(), color);
    }

    /**
     * Sets a range of LEDs in the buffer.
     *
     * @param start The index at the start of the range
     * @param end The index at the end of the range
     * @param color The color of the LEDs
     */
    public void setLED(int start, int end, Color color)
    {
        int index[] = this.validate(start, end);
        start = index[0];
        end = index[1];

        for(int i = start; i < end; i++)
            this.setLED(i, color);
    }

    /**
     * Sets all LEDs in the buffer.
     *
     * @param color The color of the LEDs
     */
    public void setLED(Color8Bit color)
    {
        this.setLED(0, this.getLength(), color);
    }

    /**
     * Sets a range of LEDs in the buffer.
     *
     * @param start The index at the start of the range
     * @param end The index at the end of the range
     * @param color The color of the LEDs
     */
    public void setLED(int start, int end, Color8Bit color)
    {
        int index[] = this.validate(start, end);
        start = index[0];
        end = index[1];

        for(int i = start; i < end; i++)
            this.setLED(i, color);
    }

    private int[] validate(int start, int end)
    {
        if(end < start)
        {
            int e = end;
            end = start;
            start = e;
        }
        if(end >= this.getLength())
            throw new IndexOutOfBoundsException(end);
        if(start < 0)
            throw new IndexOutOfBoundsException(start);
        
        return new int[]{start, end};
    }


}
