namespace LaneDetection
{
    class Sensor
    {
        public int x1, x2, y;
        public double angle = 0;
        public Sensor(int x1, int x2, int y)
        {
            this.x1 = x1;
            this.x2 = x2;
            this.y = y;
        }
    }

    class Line
    {
        public int x1, x2, y1, y2;
        public Line(int x1, int y1, int x2, int y2)
        {
            this.x1 = x1;
            this.x2 = x2;
            this.y1 = y1;
            this.y2 = y2;
        }
    }
}
