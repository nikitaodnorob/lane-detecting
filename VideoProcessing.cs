using OpenCvSharp;
using System;
using System.Collections.Generic;
using System.Linq;

namespace LaneDetection
{
    partial class Program
    {
        /// <summary>
        /// Select left or right side of current lane considering results of the previous frame
        /// </summary>
        /// <param name="candidates">Potentional lane</param>
        /// <param name="oldResult">Results of the previous frame</param>
        /// <param name="taken">Counters of taking old results</param>
        /// <returns></returns>
        static List<Sensor> SelectLineGroup(List<List<Sensor>> candidates, List<Sensor> oldResult, ref int taken)
        {
            if (candidates == null || candidates.Count == 0)
                return oldResult;
            if (candidates.Count(g => g.Count > 15) > 5 || candidates.Count(g => g.Count > 20) > 3)
                return oldResult;
            if (oldResult != null && oldResult.Count != 0 && taken < 5)
            {
                List<Sensor> candidate = candidates.Find(g => countD(g) < 0.01
                                         && Math.Abs(g.Average(s => s.angle) - oldResult.Average(s => s.angle)) < 0.05);
                if (candidate != null && candidate.Count > 10)
                {
                    taken++;
                    return candidate;
                }
            }
            taken = 0;
            return candidates.OrderBy(g => countD(g) / rank(g.Count)).First();
        }

        /// <summary>
        /// Select the groups which is the lane
        /// </summary>
        /// <param name="groups">The list of sensors' groups</param>
        /// <param name="oldResultGroups">Results of the previous frame</param>
        /// <param name="taken">Counters of taking old results</param>
        /// <returns>Result groups</returns>
        static List<List<Sensor>> SelectGroups(List<List<Sensor>> groups, List<List<Sensor>> oldResultGroups, ref int[] taken)
        {
            List<List<Sensor>> result = new List<List<Sensor>>();
            List<Sensor> leftLine = null, rightLine = null;
            int leftTaken = taken[0], rightTaken = taken[1];

            if (groups == null || groups.Count == 0) return oldResultGroups;

            var leftCandidates = groups.Where(g => g.Average(s => s.angle) >= 0).OrderBy(g => countD(g) / rank(g.Count)).ToList();
            var rightCandidates = groups.Where(g => g.Average(s => s.angle) < 0).OrderBy(g => countD(g) / rank(g.Count)).ToList();

            leftLine = SelectLineGroup(leftCandidates, oldResultGroups?[0], ref leftTaken);
            rightLine = SelectLineGroup(rightCandidates, oldResultGroups?[1], ref rightTaken);
            taken = new int[2] { leftTaken, rightTaken };

            result.Add(leftLine); result.Add(rightLine);

            return result;
        }

        static void FindLaneInTheVideo(string path)
        {
            VideoCapture capture = new VideoCapture(path);
            Mat workAreaMask = CreateMask();
            using (Window win1 = new Window("test1"))
            {
                Mat image = new Mat();
                //  We will save previous results here
                List<List<Sensor>> oldResultGroups = null;
                int[] countTaked = new int[2] { 0, 0 };
                while (true)
                {
                    DateTime dt1 = DateTime.Now;
                    capture.Read(image);
                    if (image.Empty()) break;

                    if (capture.PosFrames % 2 != 0) continue;

                    //  Get the work area
                    Mat image_s = image.SubMat( Camera.vert_frame[0], Camera.vert_frame[1],
                                                Camera.hor_frame[0], Camera.hor_frame[1]);
                    Mat workArea = new Mat();
                    image_s.CopyTo(workArea, workAreaMask);

                    //  Get HSV, grat and canny
                    Mat hsv_image = workArea.CvtColor(ColorConversionCodes.RGB2HSV);
                    Mat canny1 = hsv_image.Canny(40, 60);
                    Mat gray = workArea.CvtColor(ColorConversionCodes.BGR2GRAY);
                    Mat canny2 = gray.Canny(40, 60);
                    Mat canny = new Mat();
                    Cv2.BitwiseAnd(canny1, canny2, canny);

                    //  Get, filter and draw contours
                    Mat hsv_contoures = new Mat();
                    hsv_image.CopyTo(hsv_contoures);
                    var contoures = FindContoures(canny);
                    hsv_contoures.DrawContours(contoures, -1, Scalar.Red);

                    //  Get indexers
                    MatOfByte3 hsv_cont_ind = new MatOfByte3(hsv_contoures);
                    MatOfByte3 hsv_ind = new MatOfByte3(hsv_image);
                    var hsv_cont_indexer = hsv_cont_ind.GetIndexer();
                    var hsv_indexer = hsv_ind.GetIndexer();

                    //  Make steps of the algorithm
                    List<Sensor> sensors = GetSensors(hsv_contoures, hsv_cont_indexer);
                    List<Sensor> filteredByContours = FilterByContours(sensors, hsv_cont_indexer);
                    List<Sensor> filteredByColors = FilterByColorAndChangeColor(filteredByContours, hsv_indexer);
                    List<Sensor> filteredByNearSensors = FilterByNearSensors(filteredByColors);
                    List<List<Sensor>> groupedByAngle = GroupByAngle(filteredByNearSensors).Where(g => g.Count > 2).ToList();
                    List<List<Sensor>> groupedByDistance = GroupByDistance(groupedByAngle).Where(g => g.Count > 2).ToList();
                    List<List<Sensor>> groupedWithoudCovering = DeleteCovering(groupedByDistance);
                    List<List<Sensor>> unionGroups = UnionGroups(groupedWithoudCovering).Where(g => g.Count > 2).ToList();
                    List<List<Sensor>> resultGroups = SelectGroups(unionGroups, oldResultGroups, ref countTaked);
                    image.SaveImage("image.png");
                    //  Draw the result
                    foreach (var group in resultGroups)
                        if (group != null)
                            foreach (var line in GetLinesForGroup(group))
                                image.Line(line.x1 + Camera.hor_frame[0], line.y1 + Camera.vert_frame[0],
                                            line.x2 + Camera.hor_frame[0], line.y2 + Camera.vert_frame[0], Scalar.Blue, 5);
                    image.SaveImage("res.png");
                    Mat imageForDisplay = image.Resize(new Size(0, 0), 0.5, 0.5);
                    win1.ShowImage(imageForDisplay);
                    oldResultGroups = resultGroups;

                    DateTime dt2 = DateTime.Now;
                    Console.WriteLine("{0}\tms", (dt2 - dt1).TotalMilliseconds);

                    int key = Cv2.WaitKey(0);
                    if (key == 27) break; //escape

                    //  Free resourses
                    image_s.Release();
                    workArea.Release();
                    hsv_ind.Release();
                    hsv_cont_ind.Release();
                    gray.Release();
                    canny1.Release();
                    canny2.Release();
                    canny.Release();
                    hsv_image.Release();
                    hsv_contoures.Release();
                }
            }
        }
    }
}
