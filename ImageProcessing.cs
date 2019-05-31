using OpenCvSharp;
using System.Collections.Generic;
using System.Linq;

namespace LaneDetection
{
    partial class Program
    {
        /// <summary>
        /// Select the groups which is the lane
        /// </summary>
        /// <param name="groups">The list of sensors' groups</param>
        /// <returns>Result groups</returns>
        static List<List<Sensor>> SelectGroups(List<List<Sensor>> groups)
        {
            List<List<Sensor>> result = new List<List<Sensor>>();

            var lefts = groups.Where(g => g.Average(s => s.angle) >= 0).OrderBy(g => countD(g) / rank(g.Count)).ToList();
            var rights = groups.Where(g => g.Average(s => s.angle) < 0).OrderBy(g => countD(g) / rank(g.Count)).ToList();

            if (lefts.Count > 0)
                result.Add(lefts.First());
            if (rights.Count > 0)
                result.Add(rights.First());
            return result;
        }

        static void FindLaneInTheImage(string path)
        {
            Mat workAreaMask = CreateMask();
            using (Window win1 = new Window("test1"))
            {
                Mat image = new Mat(path);
                //  Get the work area
                Mat imageS = image.SubMat(Camera.vert_frame[0], Camera.vert_frame[1],
                                            Camera.hor_frame[0], Camera.hor_frame[1]);
                Mat workArea = new Mat();
                imageS.CopyTo(workArea, workAreaMask);

                //  Get HSV, grat and canny
                Mat hsvImage = workArea.CvtColor(ColorConversionCodes.RGB2HSV);
                Mat canny1 = hsvImage.Canny(40, 60);
                Mat gray = workArea.CvtColor(ColorConversionCodes.BGR2GRAY);
                Mat canny2 = gray.Canny(40, 60);
                Mat canny = new Mat();
                Cv2.BitwiseAnd(canny1, canny2, canny);

                //  Get, filter and draw contours
                Mat hsvContoures = new Mat();
                hsvImage.CopyTo(hsvContoures);
                var contoures = FindContoures(canny);
                hsvContoures.DrawContours(contoures, -1, Scalar.Red);

                //  Get indexers
                MatOfByte3 hsvContInd = new MatOfByte3(hsvContoures);
                MatOfByte3 hsvInd = new MatOfByte3(hsvImage);
                var hsvContIndexer = hsvContInd.GetIndexer();
                var hsvIndexer = hsvInd.GetIndexer();

                //  Make steps of the algorithm
                List<Sensor> sensors = GetSensors(hsvContoures, hsvContIndexer);
                List<Sensor> filteredByContours = FilterByContours(sensors, hsvContIndexer);
                List<Sensor> filteredByColors = FilterByColorAndChangeColor(filteredByContours, hsvIndexer);
                List<Sensor> filteredByNearSensors = FilterByNearSensors(filteredByColors);
                List<List<Sensor>> groupedByAngle = GroupByAngle(filteredByNearSensors).Where(g => g.Count > 2).ToList();
                List<List<Sensor>> groupedByDistance = GroupByDistance(groupedByAngle).Where(g => g.Count > 2).ToList();
                List<List<Sensor>> groupedWithoudCovering = DeleteCovering(groupedByDistance);
                List<List<Sensor>> unionGroups = UnionGroups(groupedWithoudCovering).Where(g => g.Count > 2).ToList();
                List<List<Sensor>> resultGroups = SelectGroups(unionGroups);

                //  Draw the result
                foreach (var group in resultGroups)
                    if (group != null)
                        foreach (var line in GetLinesForGroup(group))
                            image.Line( line.x1 + Camera.hor_frame[0], line.y1 + Camera.vert_frame[0],
                                        line.x2 + Camera.hor_frame[0], line.y2 + Camera.vert_frame[0], Scalar.Blue, 5);

                Mat imageForDisplay = image.Resize(new Size(0, 0), 0.5, 0.5);
                win1.ShowImage(imageForDisplay);
                Cv2.WaitKey(0);

                //  Free resourses
                image.Release();
                imageS.Release();
                workArea.Release();
                hsvInd.Release();
                hsvContInd.Release();
                gray.Release();
                canny1.Release();
                canny2.Release();
                canny.Release();
                hsvImage.Release();
                hsvContoures.Release();
            }
        }
    }
}
