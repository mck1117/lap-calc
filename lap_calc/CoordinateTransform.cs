using DotSpatial.Projections;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Numerics;
using System.Text;
using System.Threading.Tasks;

namespace lap_calc
{
    public interface ICoordinateConverter
    {
        Vector2 ToLocal(double lat, double lon);
    }

    internal class UtmConverter : ICoordinateConverter
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

    internal class SuperCheatyConverter : ICoordinateConverter
    {
        private static readonly double EarthCircumferenceMetersPerDegree = 40075017 / 360;

        private double LocalMetersPerLongitude { get; init; }

        private double OffsetLat { get; init; }

        private double OffsetLon { get; init; }

        public SuperCheatyConverter(double referenceLat, double referenceLon)
        {
            // The circumference of the latitude near the track won't change much, so we can precompute it
            this.LocalMetersPerLongitude = EarthCircumferenceMetersPerDegree * Math.Cos(referenceLat * Math.PI / 180);

            this.OffsetLon = referenceLon;
            this.OffsetLat = referenceLat;
        }

        public Vector2 ToLocal(double lat, double lon)
        {
            double lonDelta = lon - this.OffsetLon;
            double x = this.LocalMetersPerLongitude * lonDelta;

            double latDelta = lat - this.OffsetLat;
            double y = EarthCircumferenceMetersPerDegree * latDelta;

            return new Vector2((float)x, (float)y);
        }
    }
}
