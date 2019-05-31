using OpenCvSharp;

namespace LaneDetection
{
    /// <summary>
    /// The description of camera properties
    /// </summary>
    abstract class Camera
    {
        //  The point of horizont
        public static Point hor_point = new Point(903 - 267, 579);
        //  The vertical bounds of the frame
        public static int[] vert_frame = new int[2] { hor_point.Y, 836 };
        //  The horizontal bounds of the frame
        public static int[] hor_frame = new int[2] { 267, 1807 };
        //  The length of top side of the trapezy
        public static int hor_top_length = 200;
        //  The maximum thickness of lane line
        public static int max_lane_thickness = 40;
    }
}
