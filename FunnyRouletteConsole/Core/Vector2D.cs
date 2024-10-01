namespace FunnyRouletteConsole.Core
{
    /// <summary>
    /// Represents a 2D vector for calculations.
    /// </summary>
    public struct Vector2D
    {
        public double X { get; set; }
        public double Y { get; set; }

        public Vector2D(double x, double y)
        {
            X = x;
            Y = y;
        }

        // Vector operations
        public double Length() => Math.Sqrt(X * X + Y * Y);

        public Vector2D Normalize()
        {
            double length = Length();
            if (length == 0)
                return new Vector2D(0, 0);
            return new Vector2D(X / length, Y / length);
        }

        public double Dot(Vector2D other) => X * other.X + Y * other.Y;

        public double Cross(Vector2D other) => X * other.Y - Y * other.X;

        public static Vector2D operator -(Vector2D a, Vector2D b) =>
            new Vector2D(a.X - b.X, a.Y - b.Y);

        public static Vector2D operator +(Vector2D a, Vector2D b) =>
            new Vector2D(a.X + b.X, a.Y + b.Y);

        public static Vector2D operator *(Vector2D v, double scalar) =>
            new Vector2D(v.X * scalar, v.Y * scalar);

        public static Vector2D operator *(double scalar, Vector2D v) =>
            new Vector2D(v.X * scalar, v.Y * scalar);

        /// <summary>
        /// Calculates the distance from a point to a line segment.
        /// </summary>
        public static double DistancePointToSegment(Vector2D p, Vector2D v, Vector2D w)
        {
            // Return minimum distance between line segment vw and point p
            double l2 = (v - w).LengthSquared(); // Length squared of segment
            if (l2 == 0.0) return (p - v).Length(); // v == w case
            // Consider the line extending the segment, parameterized as v + t (w - v)
            // Find projection of point p onto the line
            double t = ((p - v).Dot(w - v)) / l2;
            t = Math.Max(0, Math.Min(1, t));
            Vector2D projection = v + (w - v) * t;
            return (p - projection).Length();
        }

        public double LengthSquared() => X * X + Y * Y;
    }
}