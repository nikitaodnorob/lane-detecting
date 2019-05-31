namespace LaneDetection
{
    partial class Program
    {
        static void Main(string[] args)
        {
            //  For example, detect lane in the image
            FindLaneInTheImage("../../examples/1_sunny/source.png");
            
            //  If you fix the CameraProperties, you can see how the algorithm works at night images
            //FindLaneInTheImage("../../examples/2_night/source.png");

            //  We can also detect lane in the videos
            //FindLaneInTheVideo("video.mp4");
        }
    }
}
