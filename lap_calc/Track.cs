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
        public FindSegmentResult CrossTrackStraightSection(Vector2 test)
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

        private static Vector3 ToV3(Vector2 x)
        {
            return new Vector3(x.X, x.Y, 0);
        }

        public FindSegmentResult CrossTrack(Vector2 test)
        {
            var prevSum = Previous.Direction + this.Direction;
            // b1 is a vector pointed to the right, mid way between the previous segment and the current segment
            var b1 = new Vector2(prevSum.Y, -prevSum.X);

            var nextSum = this.Direction + this.Next.Direction;
            // b2 is a vector pointed to the right, mid way between the current segment and the next segment
            var b2 = new Vector2(nextSum.Y, -nextSum.X);

            // dot(A, B) = |A| |B| cos(theta)
            var angleBetween = Math.Acos(Vector2.Dot(b1, b2) / (b1.Length() * b2.Length()));

            // 0.052 radians ~= 3 degrees
            if (angleBetween < 0.052) {
                // For small angles, the "linear" approximation works well enough and
                // avoids numerical stability issues of the intersection being very far away
                return CrossTrackStraightSection(test);
            }


            // Find the intersection point between vectors b1 and b2 - this is the "instant center" of the corner
            Vector2 c;

            {
                // Calculate the cross product of each line segment's direction with the other line segment's direction
                float cross3 = VectorHelper.Cross(b2, -this.Relative);
                float cross4 = VectorHelper.Cross(b2, b1 - this.Relative);

                float t = cross3 / (cross3 - cross4);
                c = this.First + new Vector2(b1.X * t, b1.Y * t);
            }

            Vector2 cToTest = test - c;
            Vector2 CA = this.First - c;
            Vector2 CB = this.Second - c;

            // this value computes distance from centerline (arc) of the segment, but doesn't account for left/right
            // turns having opposite sign. crossTrack>0 means the car is on the "outside" of the corner, whichever side
            // that is.
            // We want "positive = left" but right now it's "positive = outside"
            var crossTrack = cToTest.Length() - CB.Length();

            // For left hand corners, flip the sign on crossTrack
            if (VectorHelper.Cross(CB, this.Direction) > 0)
            {
                crossTrack *= -1;
            }

            // beta is the total angle of this track segment
            var beta = Math.Acos(Vector2.Dot(CA, CB) / (CA.Length() * CB.Length()));
            // alpha is the angle of the cars progress along the segment
            var alpha = Math.Acos(Vector2.Dot(cToTest, CA) / (CA.Length() * cToTest.Length()));

            var progressAlong = alpha / beta;

            return new FindSegmentResult { DistanceAlong = (float)(this.Length * progressAlong), FractionAlong = (float)progressAlong, CrossTrack = (float)crossTrack };
        }
    }

    public class Track
    {
        public Segment[] TrackSegments { get; init; }

        public ICoordinateConverter CoordinateConverter { get; init; }

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

        public Track(string folder)
        {
            var refPoint = File.ReadAllText(folder + "\\track.txt").Split(",").Select(s => double.Parse(s)).ToArray();
            //var converter = new UtmConverter(refPoint[0], refPoint[1]);
            this.CoordinateConverter = new SuperCheatyConverter(refPoint[0], refPoint[1]);

            var trackPoints = FileLines(folder + "\\points.csv")
                .Select(l => l.Split(','))
                .Select(s => this.CoordinateConverter.ToLocal(double.Parse(s[0].Trim()), double.Parse(s[1].Trim())));

            (this.TrackSegments, this.Length) = MakeTrack(trackPoints);
        }

        public Track(IEnumerable<Vector2> trackPoints)
        {
            (this.TrackSegments, this.Length) = MakeTrack(trackPoints);
        }

        private static (Segment[], float) MakeTrack(IEnumerable<Vector2> trackPoints)
        {
            var trackSegments = trackPoints.Zip(trackPoints.Skip(1), (a, b) => new Segment(a, b)).ToArray();

            // Compute segment start distances
            float totalLength = 0;
            for (int i = 0; i < trackSegments.Length; i++)
            {
                trackSegments[i].StartDistance = totalLength;
                totalLength += trackSegments[i].Length;
            }

            // Build linked list of track segments
            for (int i = 0; i < trackSegments.Length - 1; i++)
            {
                var cur = trackSegments[i];
                var next = trackSegments[i + 1];

                cur.Id = i;
                cur.Next = next;
                next.Previous = cur;
            }

            // link ends together
            {
                var last = trackSegments[trackSegments.Length - 1];
                trackSegments[0].Previous = last;
                last.Next = trackSegments[0];
                last.Id = trackSegments.Length - 1;
            }

            return (trackSegments, totalLength);
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

                if (afterLastSector && beforeNextSector)
                {
                    //var trackFraction = ((current.StartDistance + current.Length * crossTrackCurrent.FractionAlong) / track.Length);
                    //Console.WriteLine("iter " + (i + 1) + " pos " + trackFraction + " cross " + crossTrackCurrent.CrossTrack + " cross 2 " + crossTrackCurrentBoth.Item2.CrossTrack);
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
