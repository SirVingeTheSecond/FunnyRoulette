namespace FunnyRouletteConsole.Core
{
    /// <summary>
    /// Represents a fret (divider) on the roulette wheel.
    /// </summary>
    public class Fret
    {
        public double Angle { get; set; } // Angular position in degrees
        public double RadiusInner { get; set; } = 0.8; // Inner radius (normalized)
        public double RadiusOuter { get; set; } = 1.0; // Outer radius (normalized)

        public Fret(double angle)
        {
            Angle = angle;
        }

        /// <summary>
        /// Gets the start and end points of the fret line in Cartesian coordinates, considering wheel rotation.
        /// </summary>
        public (Vector2D Start, Vector2D End) GetLinePoints(double wheelPosition)
        {
            double angleRad = (Angle + wheelPosition) * Math.PI / 180.0;

            // Start point (inner radius)
            var startX = RadiusInner * Math.Cos(angleRad);
            var startY = RadiusInner * Math.Sin(angleRad);

            // End point (outer radius)
            var endX = RadiusOuter * Math.Cos(angleRad);
            var endY = RadiusOuter * Math.Sin(angleRad);

            return (new Vector2D(startX, startY), new Vector2D(endX, endY));
        }
    }
}