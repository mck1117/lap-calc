using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using System.Numerics;
using System.Text;
using System.Text.RegularExpressions;
using System.Threading.Tasks;

namespace lap_calc
{
    public struct CarPoint
    {
        public Vector2 Position;
        public double Timestamp;
    }

    public struct LogLineRead
    {
        public string RawLine;
        public CarPoint ParsedCarPoint;
    }

    internal class LapReader
    {
        private static int FindColumnIndex(string[] headers, string name)
        {
            var matches = headers
                .Select((s, i) => (Header: s, Idx: i))
                .Where(h => h.Header.ToLowerInvariant().Contains(name.ToLowerInvariant()))
                .Select(h => h.Idx).ToArray();

            if (matches.Length != 1)
            {
                throw new ArgumentException("header match fail");
            }

            return matches[0];
        }

        private static readonly string Iso8601Regex = "^[0-9]{4}-[0-9]{2}-[0-9]{2}T([0-9]{2}):([0-9]{2}):([0-9\\.]+)Z$";

        private static double ParseIso8601(string timestamp)
        {
            var matches = Regex.Match(timestamp, Iso8601Regex);

            double hours = double.Parse(matches.Groups[1].Value);
            double minutes = double.Parse(matches.Groups[2].Value);
            double seconds = double.Parse(matches.Groups[3].Value);

            return seconds + 60 * (minutes + (60 * hours));
        }

        private static Func<string[], float> MakeTimestampParser(string[] headers, string[] sampleLine)
        {
            try
            {
                // Racecapture uses "interval", plain milliseconds
                int timestampCol = FindColumnIndex(headers, "interval");

                // make sure it looks like a number
                if (Regex.IsMatch(sampleLine[timestampCol], "[0-9\\.]+"))
                {
                    return (line) => float.Parse(line[timestampCol]) / 1000.0f;
                }
            } catch { }

            try
            {
                // Look for something called timestamp
                int timestampCol = FindColumnIndex(headers, "timestamp");
                string exampleTimestamp = sampleLine[timestampCol].Trim();

                // Check if it looks like an ISO 8601 datetime, YYYY-MM-DDThh:mm:ss.sZ
                string iso8601reg = "^[0-9]{4}-[0-9]{2}-[0-9]{2}T([0-9]{2}):([0-9]{2}):([0-9\\.]+)Z$";
                if (Regex.IsMatch(exampleTimestamp, iso8601reg))
                {
                    double startTimestamp = ParseIso8601(exampleTimestamp);

                    return (line) => (float)(ParseIso8601(line[timestampCol].Trim()) - startTimestamp);
                }
            } catch { }

            // couldn't decode timestamp column
            throw new InvalidDataException();
        }

        public static IEnumerable<LogLineRead> ReadLaps(String file, ICoordinateConverter converter)
        {
            using (StreamReader sr = new StreamReader(file))
            {
                var header = sr.ReadLine().Split(',');

                int latCol = FindColumnIndex(header, "lat");
                int lonCol = FindColumnIndex(header, "lon");
                Func<string[], float> timestampParser = null;

                while (true)
                {
                    string line = sr.ReadLine();

                    if (line == null)
                    {
                        yield break;
                    }
                    else
                    {
                        var lineSplit = line.Split(',');

                        string latStr = lineSplit[latCol].Trim();
                        string lonStr = lineSplit[lonCol].Trim();

                        if (latStr.Length == 0 || lonStr.Length == 0)
                        {
                            // This record has no coordinates (some other channel logs faster than lat/lon), so skip it
                            continue;
                        }

                        if (timestampParser == null)
                        {
                             timestampParser = MakeTimestampParser(header, lineSplit);
                        }

                        float timestamp = timestampParser(lineSplit);

                        yield return new LogLineRead
                        {
                            RawLine = line,
                            ParsedCarPoint = new CarPoint
                            {
                                Position = converter.ToLocal(double.Parse(latStr), double.Parse(lonStr)),
                                Timestamp = timestamp
                            }
                        };
                    }
                }
            }
        }
    }
}
