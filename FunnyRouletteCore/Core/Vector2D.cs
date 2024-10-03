namespace FunnyRouletteCore.Core
{
    /// <summary>
    /// Represents a 2D vector.
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

        public double Length() => Math.Sqrt(X * X + Y * Y);
        public Vector2D Normalize()
        {
            double length = Length();
            return length == 0 ? new Vector2D(0, 0) : new Vector2D(X / length, Y / length);
        }

        public double Dot(Vector2D other) => X * other.X + Y * other.Y;
        public double Cross(Vector2D other) => X * other.Y - Y * other.X;

        public static Vector2D operator -(Vector2D a, Vector2D b) => new Vector2D(a.X - b.X, a.Y - b.Y);
        public static Vector2D operator +(Vector2D a, Vector2D b) => new Vector2D(a.X + b.X, a.Y + b.Y);
        public static Vector2D operator *(Vector2D v, double scalar) => new Vector2D(v.X * scalar, v.Y * scalar);
        public static Vector2D operator *(double scalar, Vector2D v) => new Vector2D(v.X * scalar, v.Y * scalar);

        public static double Distance(Vector2D a, Vector2D b) => (a - b).Length();
    }
}