using System;
using System.Collections.Generic;
using System.Linq;
using System.Numerics;
using System.IO;

using DotSpatial.Projections;
using DotSpatial.Projections.Transforms;

namespace lap_calc
{
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
            var t = new Track(args[0]);
            TrackPositionFinder tpf = new TrackPositionFinder(t);

            var carPoints = LogReader.ReadLog(args[1], t.CoordinateConverter);

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
