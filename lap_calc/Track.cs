using System;
using System.Collections.Generic;
using System.Numerics;
using System.Linq;
using System.IO;

namespace lap_calc
{
    public struct FindSegmentResult
    {
        public float DistanceAlong { get; init; }
        public float FractionAlong { get; init; }

        // positive cross track means vehicle is left of centerline
        public float CrossTrack { get; init; }

        public override string ToString()
        {
            return "frac: " + FractionAlong + " cross: " + CrossTrack;
        }
    }

    class VectorHelper
    {
        public static float Cross(Vector2 a, Vector2 b)
        {
            return a.X * b.Y - a.Y * b.X;
        }
    }

    public class Segment
    {
        public Vector2 First { get; init; }
        public Vector2 Second { get; init; }

        public Vector2 Direction { get; init; }
        public Vector2 Relative { get; init; }

        public Segment Previous { get; set; }
        public Segment Next { get; set; }

        public float Length { get; init; }

        public float StartDistance { get; set; }

        public int Id { get; set; } = -1;

        public Segment(Vector2 first, Vector2 second)
        {
            this.First = first;
            this.Second = second;

            Relative = this.Second - this.First;
            Direction = Vector2.Normalize(this.Relative);
            Length = Relative.Length();
        }

        public override string ToString()
        {
            return "length " + Length;
        }

        // Computes the cross track error (X) and forward distance (Y) from the first point.
        public FindSegmentResult CrossTrack(Vector2 test)
        {
            var startToPoint = test - First;

            // crossProduct = sin(theta)
            // theta = angle between track segment and (vehicle - start) vector
            var crossProduct = VectorHelper.Cross(this.Relative, startToPoint);

            var theta = Math.Atan2(crossProduct, Vector2.Dot(this.Relative, startToPoint));

            float distanceFromStart = startToPoint.Length();
            float scalarLength = distanceFromStart * (float)Math.Cos(theta);
            float crossTrack = distanceFromStart * (float)Math.Sin(theta);

            return new FindSegmentResult { DistanceAlong = scalarLength, FractionAlong = scalarLength / Length, CrossTrack = crossTrack };
        }
    }

    public class Track
    {
        public Segment[] TrackSegments { get; init; }

        public float Length { get; init; }

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

        public static (Track, UtmConverter) Load(String folder)
        {
            var refPoint = File.ReadAllText(folder + "\\track.txt").Split(",").Select(s => double.Parse(s)).ToArray();
            UtmConverter converter = new UtmConverter(refPoint[0], refPoint[1]);

            var trackPoints = FileLines(folder + "\\points.csv")
                .Select(l => l.Split(',').Select(s => s.Trim()).ToArray())
                .Select(s => converter.ToLocal(double.Parse(s[0]), double.Parse(s[1]))).ToArray();

            return (new Track(trackPoints), converter);
        }

        public Track(IEnumerable<Vector2> trackPoints)
        {
            TrackSegments = trackPoints.Zip(trackPoints.Skip(1), (a, b) => new Segment(a, b)).ToArray();

            // Compute segment start distances
            float totalLength = 0;
            for (int i = 0; i < TrackSegments.Length; i++)
            {
                TrackSegments[i].StartDistance = totalLength;
                totalLength += TrackSegments[i].Length;
            }

            this.Length = totalLength;

            // Build linked list of track segments
            for (int i = 0; i < TrackSegments.Length - 1; i++)
            {
                var cur = TrackSegments[i];
                var next = TrackSegments[i + 1];

                cur.Id = i;
                cur.Next = next;
                next.Previous = cur;
            }

            // link ends together
            {
                var last = TrackSegments[TrackSegments.Length - 1];
                TrackSegments[0].Previous = last;
                last.Next = TrackSegments[0];
                last.Id = TrackSegments.Length - 1;
            }
        }
    }

    public struct FindPositionResult
    {
        public float TrackFraction;
        public float DistanceFromStart;
        public float CrossTrackPosition;
    }

    public class TrackPositionFinder
    {
        private class FindPositionResult2
        {
            public readonly FindPositionResult Result;
            public readonly Segment s;

            public FindPositionResult2(Segment s, FindSegmentResult crossTrack, Track t)
            {
                float trackLength = t.Length;

                float distanceAlongTrack = s.StartDistance + crossTrack.FractionAlong * s.Length;
                float trackFraction = distanceAlongTrack / trackLength;

                Result = new FindPositionResult { TrackFraction = trackFraction, DistanceFromStart = distanceAlongTrack, CrossTrackPosition = crossTrack.CrossTrack };

                this.s = s;
            }
        }

        private readonly Track track;

        public TrackPositionFinder(Track track)
        {
            this.track = track;
        }

        private Segment lastPosition = null;

        private FindPositionResult2 FindPositionImpl(Vector2 point)
        {
            // bootstrap if lost on track
            if (lastPosition == null)
            {
                lastPosition = track.TrackSegments[0];
            }

            Segment current = lastPosition;

            for (int i = 0; i < track.TrackSegments.Length + 5; i++)
            {
                // 3 conditions:
                // 1. cross track error to this segment is reasonable
                // 2. ahead of previous segment
                // 3. behind next segment

                // If the cross track error or frac is huge, we're in the wrong part of the track.
                var crossTrackCurrent = current.CrossTrack(point);
                if (
                    crossTrackCurrent.CrossTrack > 20 || crossTrackCurrent.CrossTrack < -20 ||
                    crossTrackCurrent.FractionAlong > 2 || crossTrackCurrent.FractionAlong < -1
                    )
                {
                    current = current.Next;
                    continue;
                }

                if (crossTrackCurrent.FractionAlong == 0)
                {
                    // Congrats, you EXACTLY hit the start of a sector
                    return new FindPositionResult2(current, crossTrackCurrent, this.track);
                }

                var prevSum = current.Previous.Direction + current.Direction;
                // b1 is a vector pointed to the right, mid way between the previous segment and the current segment
                var b1 = Vector2.Normalize(new Vector2(prevSum.Y, -prevSum.X));
                bool afterLastSector = VectorHelper.Cross(point - current.First, b1) < 0;

                var nextSum = current.Direction + current.Next.Direction;
                // b2 is a vector pointed to the right, mid way between the current segment and the next segment
                var b2 = Vector2.Normalize(new Vector2(nextSum.Y, -nextSum.X));
                bool beforeNextSector = VectorHelper.Cross(point - current.Second, b2) > 0;

                bool isFound = afterLastSector && beforeNextSector;

                

                if (isFound)
                {
                    // var trackFraction = ((current.StartDistance + current.Length * crossTrackCurrent.FractionAlong) / track.Length);
                    // Console.WriteLine("iter " + (i + 1) + " pos " + trackFraction + " cross " + crossTrackCurrent.CrossTrack);
                    return new FindPositionResult2(current, crossTrackCurrent, this.track);
                }

                // Search forward from the current point, hopefully you're driving around the track forward...
                current = current.Next;
            }

            // You aren't on the track, no segment is valid.
            return null;
        }

        public FindPositionResult? FindPosition(Vector2 point)
        {
            var result = FindPositionImpl(point);

            if (result != null)
            {
                //Console.WriteLine(result.s.Id + "\t" + result.Result.TrackFraction + "\t" + result.Result.DistanceFromStart + "\t" + result.Result.CrossTrackPosition);
            }
            else
            {
                //Console.WriteLine("null");
            }

            // Store last resolved segment to speed up next search
            this.lastPosition = result?.s;

            return result?.Result;
        }

        public string FindCornerName(FindPositionResult position)
        {
            if (position.DistanceFromStart < 554) return "1";
            if (position.DistanceFromStart < 684) return "2";
            if (position.DistanceFromStart < 780) return "3";
            if (position.DistanceFromStart < 879) return "4";
            if (position.DistanceFromStart < 1157) return "5";
            if (position.DistanceFromStart < 1621) return "6";
            if (position.DistanceFromStart < 1788) return "7";
            if (position.DistanceFromStart < 1895) return "8";    // 8a
            if (position.DistanceFromStart < 2010) return "8";    // 8b
            if (position.DistanceFromStart < 2238) return "9";
            if (position.DistanceFromStart < 2455) return "10";
            if (position.DistanceFromStart < 2687) return "11";
            if (position.DistanceFromStart < 2999) return "12";
            if (position.DistanceFromStart < 3249) return "13";
            if (position.DistanceFromStart < 3332) return "14";
            if (position.DistanceFromStart < 3493) return "15";
            return "16";
        }
    }
}
