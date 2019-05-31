using System;
using System.Collections.Generic;
using System.Linq;
using OpenCvSharp;

namespace LaneDetection
{
    partial class Program
    {
        /// <summary>
        /// Create the binary mask for work area taking
        /// </summary>
        /// <returns>The mask for work area</returns>
        static Mat CreateMask()
        {
            int width = Camera.hor_frame[1] - Camera.hor_frame[0];
            int height = Camera.vert_frame[1] - Camera.vert_frame[0];
            Mat mskMat = Mat.Zeros(new Size(width, height), MatType.CV_8U);
            //  Draw a trapeze
            List<Point> trapeze = new List<Point>();
            trapeze.Add(new Point(0, height));
            trapeze.Add(new Point(width, height));
            trapeze.Add(new Point(Camera.hor_point.X + Camera.hor_top_length / 2, 0));
            trapeze.Add(new Point(Camera.hor_point.X - Camera.hor_top_length / 2, 0));
            //  Fill the trapeze by white color
            mskMat.FillPoly(new List<List<Point>> { trapeze }, Scalar.White);
            return mskMat;
        }

        //  Color constants in the BGR format
        static Vec3b COLOR_RED = new Vec3b(0, 0, 255);
        static Vec3b COLOR_BLACK = new Vec3b(0, 0, 0);

        //  Calculate the thickness of the lane line in given Y position by geometry rules
        static int CalculateLaneThickness(int workplaceY)
        {
            double h = Camera.vert_frame[1] - Camera.vert_frame[0];
            double h1 = workplaceY;
            double th = Camera.max_lane_thickness;
            return (int)(th * h1 / h);
        }

        /// <summary>
        /// Create virtual sensors for specified image
        /// </summary>
        /// <param name="hsv">The work area in HSV</param>
        /// <param name="indexer">The indexer of work area in HSV (for fast access to pixels' values)</param>
        /// <returns>The list of sensors</returns>
        static List<Sensor> GetSensors(Mat hsv, MatIndexer<Vec3b> indexer)
        {
            List<Sensor> result = new List<Sensor>();
            for (int y = hsv.Height - 5; y >= 0; y -= 3)
            {
                int laneThickness = CalculateLaneThickness(y);
                //  Don't create sensors if the lane thickness too small
                if (laneThickness < 10) break;

                for (int x = 0; x < hsv.Width - laneThickness - 10; x += 5)
                {
                    //  Don't create sensors if its start is out of trapeze range
                    if (indexer[y, x] == COLOR_BLACK && indexer[y, x + 1] == COLOR_BLACK && indexer[y, x + 2] == COLOR_BLACK)
                        continue;
                    result.Add(new Sensor(x, x + laneThickness + 10, y));
                }
            }
            return result;
        }

        /// <summary>
        /// Filter the contours which was found by Canny algorithm
        /// </summary>
        /// <param name="canny">The result of Canny detector work</param>
        /// <returns>Filtered contours</returns>
        static List<Point[]> FindContoures(Mat canny)
        {
            Point[][] contoures = Cv2.FindContoursAsArray(canny, RetrievalModes.External, ContourApproximationModes.ApproxSimple);
            List<Point[]> contoures2 = new List<Point[]>();
            foreach (var cont in contoures)
            {
                if (Math.Abs(Cv2.ContourArea(cont)) < 2) continue;  // square
                if (Cv2.ArcLength(cont, true) < 15) continue;       // perimeter
                if (cont.Length < 5) continue;                      // number of pixels in the contour

                contoures2.Add(cont);
            }
            return contoures2;
        }

        /// <summary>
        /// Select sensors which cover contours from both sides
        /// </summary>
        /// <param name="sensors">The list of sensors</param>
        /// <param name="contIndexer">The indexer for HSV image of work area</param>
        /// <returns>Filtered sensors</returns>
        static List<Sensor> FilterByContours(List<Sensor> sensors, MatIndexer<Vec3b> contIndexer)
        {
            List<Sensor> filteredByContours = new List<Sensor>();
            foreach (var sensor in sensors)
            {
                int cnt = 0;
                for (int x = sensor.x1; x <= sensor.x1 + 10; x++)
                {
                    if (contIndexer[sensor.y, x] == COLOR_RED)
                    {
                        cnt++;
                        break;
                    }
                }
                for (int x = sensor.x2; x >= sensor.x2 - 10; x--)
                {
                    if (contIndexer[sensor.y, x] == COLOR_RED)
                    {
                        cnt++;
                        break;
                    }
                }
                if (cnt == 2) filteredByContours.Add(sensor);
            }
            return filteredByContours;
        }

        /// <summary>
        /// Check that the specified color is not a lane
        /// </summary>
        static bool isBadColor(Vec3b color_t)
        {
            RGB rgb_color = new RGB(color_t[2], color_t[1], color_t[0]);
            HSV hsv_color = RGB.ToHSV(rgb_color);
            return hsv_color.V > 0.85 || hsv_color.S > 0.94;
        }

        /// <summary>
        /// Filter sensors by count of pixels with bad color and changing of average color
        /// </summary>
        /// <param name="sensors">The list of sensors</param>
        /// <param name="indexer">The indexer for HSV image of work area</param>
        /// <returns>Filtered sensors</returns>
        static List<Sensor> FilterByColorAndChangeColor(List<Sensor> sensors, MatIndexer<Vec3b> indexer)
        {
            List<Sensor> result = new List<Sensor>();
            foreach (var sensor in sensors)
            {
                //  Count the average components of HSV inside the sensor
                int s0Sum = 0, v0Sum = 0, badColorsCnt = 0;
                for (int x = sensor.x1; x <= sensor.x2; x++)
                {
                    Vec3b color = indexer[sensor.y, x];
                    if (isBadColor(color)) badColorsCnt++;
                    if (badColorsCnt == 10) break;
                    s0Sum += color[1];
                    v0Sum += color[2];
                }

                //  Delete sensors where much of pixels with bad color
                if (badColorsCnt == 10) continue;

                double n = sensor.x2 - sensor.x1;
                double s0Avg = s0Sum / n, v0Avg = v0Sum / n;

                //  Count the average components of HSV outside the sensor
                int sSum = 0, vSum = 0;
                for (int x = sensor.x1 - 10; x < sensor.x1; x++)
                {
                    Vec3b color = indexer[sensor.y, x];
                    sSum += color[1];
                    vSum += color[2];
                }
                for (int x = sensor.x2 + 1; x <= sensor.x2 + 10; x++)
                {
                    Vec3b color = indexer[sensor.y, x];
                    sSum += color[1];
                    vSum += color[2];
                }
                double sAvg = sSum / 20.0, vAvg = vSum / 20.0;

                //  If difference between these values much, it may be a lane
                if (Math.Abs(s0Avg - sAvg) > 4 || Math.Abs(v0Avg - vAvg) > 9)
                    result.Add(sensor);
            }
            return result;
        }

        /// <summary>
        /// Filter sensors by existing other sensors near
        /// </summary>
        /// <param name="sensors">The list of sensors</param>
        /// <returns>Filtered sensors</returns>
        static List<Sensor> FilterByNearSensors(List<Sensor> sensors)
        {
            List<Sensor> result = new List<Sensor>();
            foreach (var sensor in sensors)
            {
                if (sensors.Exists(s => s != sensor &&
                                        Math.Abs(s.y - sensor.y) < 10 &&
                                        Math.Abs(s.x1 - sensor.x1) < 11 &&
                                        s.y != sensor.y))
                    result.Add(sensor);
            }
            return result;
        }

        /// <summary>
        /// Calculate the angle of specified sensor
        /// </summary>
        /// <param name="sensor">The sensor</param>
        /// <returns>The angle</returns>
        static double CalculateSensorAngle(Sensor sensor)
        {
            int sensorMiddleX = (sensor.x1 + sensor.x2) / 2;
            int horD = Camera.hor_point.X - sensorMiddleX;
            double tan = sensor.y * 1.0 / horD;
            return Math.Atan(tan);
        }

        /// <summary>
        /// Group sensors by angles
        /// </summary>
        /// <param name="sensors">The list of sensors</param>
        /// <returns>The list of sensors' groups</returns>
        static List<List<Sensor>> GroupByAngle(List<Sensor> sensors)
        {
            if (sensors.Count == 0) return new List<List<Sensor>> { sensors };
            foreach (var sensor in sensors)
                sensor.angle = CalculateSensorAngle(sensor);
            sensors.Sort((s1, s2) => s1.angle.CompareTo(s2.angle));

            List<List<Sensor>> groups = new List<List<Sensor>>();
            groups.Add(new List<Sensor> { sensors[0] });
            double oldAngle = sensors[0].angle;
            int currentGroupIndex = 0;
            for (int i = 1; i < sensors.Count; i++)
            {
                if (sensors[i].angle - oldAngle > 0.05)
                {
                    groups.Add(new List<Sensor>());
                    currentGroupIndex++;
                    oldAngle = sensors[i].angle;
                }
                groups[currentGroupIndex].Add(sensors[i]);
            }
            return groups;
        }

        /// <summary>
        /// Group sensors by distances to other sensors in group
        /// </summary>
        /// <param name="sensors">The list of sensors' groups</param>
        /// <returns>The list of new sensors' groups</returns>
        static List<List<Sensor>> GroupByDistance(List<List<Sensor>> groups)
        {
            List<List<Sensor>> newGroupsX = new List<List<Sensor>>();
            int currentGroupIndex = -1;
            for (int i = 0; i < groups.Count; i++)
            {
                var group = groups[i];
                group.Sort((s1, s2) => s1.x1.CompareTo(s2.x1));
                newGroupsX.Add(new List<Sensor> { group[0] });
                int oldX1 = group[0].x1;
                currentGroupIndex++;
                for (int j = 1; j < group.Count; j++)
                {
                    if (Math.Abs(group[j].x1 - oldX1) > 25)
                    {
                        newGroupsX.Add(new List<Sensor>());
                        currentGroupIndex++;
                    }
                    newGroupsX[currentGroupIndex].Add(group[j]);
                    oldX1 = group[j].x1;
                }
            }
            return newGroupsX;
        }

        /// <summary>
        /// Delete sensors which cover each other
        /// </summary>
        /// <param name="groups">The list of sensors' groups</param>
        /// <returns>The list of new sensors' groups</returns>
        static List<List<Sensor>> DeleteCovering(List<List<Sensor>> groups)
        {
            List<List<Sensor>> newGroups = new List<List<Sensor>>();
            for (int i = 0; i < groups.Count; i++)
            {
                newGroups.Add(new List<Sensor>());
                var group = groups[i];
                var groupsByX = group.GroupBy(s => s.y);
                foreach (var groupping in groupsByX)
                {
                    int cnt = groupping.Count();
                    if (cnt == 1) newGroups[i].Add(groupping.First());
                    else
                    {
                        //  If few sensors exist in the one Y position which cover each other, it will be selected the sensor,
                        //  which angle has the smallest difference to middle angle in the group
                        double avgAngle = groupping.Average(s => s.angle);
                        newGroups[i].Add(groupping.OrderBy(s => (s.angle - avgAngle) * (s.angle - avgAngle)).First());
                    }
                }
            }
            return newGroups;
        }

        /// <summary>
        /// Count the deviation sum of the group angles from middle angle
        /// </summary>
        /// <param name="group">The group of sensor</param>
        /// <returns>The deviation</returns>
        static double countD(List<Sensor> group)
        {
            double M = group.Average(s => s.angle);
            return group.Select(s => (s.angle - M) * (s.angle - M)).Sum();
        }

        /// <summary>
        /// Count the rank of sensors amount in the group
        /// </summary>
        /// <param name="n">Amount of sensors</param>
        /// <returns>The rank</returns>
        static double rank(int n)
        {
            if (n < 5) return n / 2.0;
            else if (n >= 5 && n <= 10) return 4 * n;
            else if (n >= 11 && n <= 15) return 16 * n;
            else if (n >= 16 && n <= 20) return 64 * n;
            else return 4 * n * n;
        }

        /// <summary>
        /// Union different group with similar angle deviations
        /// </summary>
        /// <param name="groups">The list of sensors' groups</param>
        /// <returns>The list of new sensors' groups</returns>
        static List<List<Sensor>> UnionGroups(List<List<Sensor>> groups)
        {
            List<List<Sensor>> newGroups = new List<List<Sensor>>();
            foreach (var group in groups.OrderByDescending(g => countD(g)))
            {
                int index = newGroups.FindIndex(g => Math.Abs(g.Average(s => s.angle) - group.Average(s => s.angle)) < 0.05 &&
                                                     countD(g) < 0.05);
                if (index > -1)
                    newGroups[index] = newGroups[index].Concat(group).ToList();
                else newGroups.Add(group);
            }
            return DeleteCovering(newGroups);
        }

        /// <summary>
        /// Get the lines to draw the lane line
        /// </summary>
        /// <param name="group">The sensors' group</param>
        /// <returns>The list of lines</returns>
        static List<Line> GetLinesForGroup(List<Sensor> group)
        {
            List<Line> result = new List<Line>();
            var sortedGroup = group.OrderByDescending(s => s.y);
            var first = sortedGroup.First();
            var last = sortedGroup.Last();
            result.Add(new Line((first.x1 + first.x2) / 2, first.y, (last.x1 + last.x2) / 2, last.y));

            return result;
        }
    }
}