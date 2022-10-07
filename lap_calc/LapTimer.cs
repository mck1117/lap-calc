using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Xml.Schema;

namespace lap_calc
{
    public struct LapTimeResult
    {
        public Double LapElapsed;
        public int LapNumber;
        public Double LastLapTime;
    }

    public class LapTimer
    {
        private double lapStartTime = 0;

        private int lapNumber = 0;

        private float lastLapFraction = 0;
        private double lastSampleTime = 0;

        private bool hasStarted = false;

        private double lastLapTime = 0;

        public LapTimeResult ProcessSegment(FindPositionResult trackPosition, double currentTime)
        {
            // We just crossed the start/finish - process that
            if (trackPosition.TrackFraction < 0.1f && lastLapFraction > 0.9f)
            {
                // Interpolate exactly when we crossed the start/finish line

                // How far from start/finish was the last sample
                float lastResidual = 1 - this.lastLapFraction;
                
                // Total distance between the last position and the current position
                float lastInterval = lastResidual + trackPosition.TrackFraction;

                // How far between the last two segments were we when we crossed the line?
                float startFinishFrac = lastResidual / lastInterval;

                // Interpolate between the last point and current point to determine the instant we crossed the line
                double crossTime = (lastSampleTime * startFinishFrac) + (currentTime * (1 - startFinishFrac));

                if (hasStarted)
                {
                    // Only compute laps after we've crossed once
                    double lapSeconds = crossTime - lapStartTime;
                    int min = (int)(lapSeconds / 60);
                    double sec = lapSeconds % 60;

                    lapNumber++;

                    lastLapTime = lapSeconds;

                    Console.WriteLine($"LAP {lapNumber} {min}:{sec:00.000}");
                }
                else
                {
                    hasStarted = true;
                    lapNumber = 0;

                    Console.WriteLine("First lap started at " + crossTime);
                }

                lapStartTime = crossTime;
            }

            this.lastLapFraction = trackPosition.TrackFraction;
            this.lastSampleTime = currentTime;


            double lapElapsed = hasStarted ? currentTime - this.lapStartTime : 0;

            return new LapTimeResult { LapElapsed = lapElapsed, LapNumber = this.lapNumber, LastLapTime = this.lastLapTime };
        }
    }
}
