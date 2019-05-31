using System;

namespace LaneDetection
{
    class HSV
    {
        public double H, S, V;

        public HSV(double H, double S, double V)
        {
            this.H = H;
            this.S = S;
            this.V = V;
        }
    }

    class RGB
    {
        public int R, G, B;

        public RGB(int R, int G, int B)
        {
            this.R = R;
            this.G = G;
            this.B = B;
        }

        public static HSV ToHSV(RGB color)
        {
            int max = Math.Max(color.R, Math.Max(color.G, color.B));
            int min = Math.Min(color.R, Math.Min(color.G, color.B));

            double hue = 0;
            if (min == max) hue = 0;
            else if (max == color.R && color.G >= color.B) hue = 60d * (color.G - color.B) / (max - min);
            else if (max == color.R && color.G < color.B) hue = 60d * (color.G - color.B) / (max - min) + 360;
            else if (max == color.G) hue = 60d * (color.B - color.R) / (max - min) + 120;
            else if (max == color.B) hue = 60d * (color.R - color.G) / (max - min) + 240;

            double saturation = (max == 0) ? 0 : 1d - (1d * min / max);
            double value = max / 255d;

            HSV res = new HSV(hue, saturation, value);
            return res;
        }
    }
}
