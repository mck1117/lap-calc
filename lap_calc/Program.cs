using System;
using System.Collections.Generic;
using System.Linq;
using System.Numerics;
using System.IO;

using DotSpatial.Projections;
using DotSpatial.Projections.Transforms;

namespace lap_calc
{
    class UtmConverter
    {
        private double OffsetX { get; init; }
        private double OffsetY { get; init; }

        public UtmConverter(double referenceLat, double referenceLon)
        {
            var offset = Project(referenceLat, referenceLon);
            OffsetX = offset[0];
            OffsetY = offset[1];
        }

        private double[] Project(double lat, double lon)
        {
            var wgs84 = KnownCoordinateSystems.Geographic.World.WGS1984;
            var utm = KnownCoordinateSystems.Projected.UtmWgs1984.WGS1984UTMZone10N;

            double[] xy = new double[] { lon, lat };

            Reproject.ReprojectPoints(xy, new double[] { 0 }, wgs84, utm, 0, 1);

            return xy;
        }

        public Vector2 ToLocal(double lat, double lon)
        {
            var raw = Project(lat, lon);

            return new Vector2((float)(raw[0] - OffsetX), (float)(raw[1] - OffsetY));
        }
    }

    class Program
    {
        public static IEnumerable<string> FileLines(string file)
        {
            using (StreamReader sr = new StreamReader(file))
            {
                while (true)
                {
                    string line = sr.ReadLine();

                    if (line == null)
                    {
                        yield break;
                    }
                    else
                    {
                        yield return line;
                    }
                }
            }
        }

        static void Main(string[] args)
        {
            (var t, var converter) = Track.Load(args[0]);
            TrackPositionFinder tpf = new TrackPositionFinder(t);

            //var trackPoints = new Vector2[]
            //{
            //    new Vector2(0, 0),
            //    new Vector2(1, 0.05f),
            //    new Vector2(2, 0.2f),
            //    new Vector2(3, 1),
            //    new Vector2(4, 2),
            //    new Vector2(4.8f, 3),
            //    new Vector2(4.95f, 4),
            //    new Vector2(5, 5),
            //};

            //var trackPoints = new Vector2[]
            //{
            //    new Vector2(0, 0),
            //    new Vector2(1, 0),
            //    new Vector2(1, 1),
            //    new Vector2(1, 2),
            //};


            // just NE of the start line
            //var testPoint = converter.ToLocal(47.25478646934098, -123.19316361737613);

            // just after chicane entry, left side of track
            //var testPoint = converter.ToLocal(47.25475119943449, -123.1897112294538);

            // just after chicane entry, right side of track
            //var testPoint = converter.ToLocal(47.25465298282364, -123.1896988431295);

            // Just before finish line
            //var testPoint = converter.ToLocal(47.254737819550414, -123.19341799646936);

            //var r = FindTrackPosition2(testPoint);

            //FindTrackPosition2(new Vector2(0.8f, 0.9f));

            var carPoints = LapReader.ReadLaps(args[1], converter);

            StreamWriter sw = new StreamWriter(args[2]);

            LapTimer timer = new LapTimer();

            foreach (var carPoint in carPoints)
            {
                var result = tpf.FindPosition(carPoint.ParsedCarPoint.Position);
                if (result != null)
                {
                    LapTimeResult laptime = timer.ProcessSegment(result.Value, carPoint.ParsedCarPoint.Timestamp);

                    string cornerName = tpf.FindCornerName(result.Value);

                    sw.WriteLine(carPoint.RawLine + "," + result.Value.DistanceFromStart + "," + result.Value.CrossTrackPosition + "," + carPoint.ParsedCarPoint.Position.X + "," + carPoint.ParsedCarPoint.Position.Y + "," + laptime.LapNumber + "," + laptime.LapElapsed + "," + laptime.LastLapTime + "," + cornerName);
                } else
                {
                    sw.WriteLine(carPoint.RawLine + ",,,,,,,,");
                }
            }
        }
    }
}
