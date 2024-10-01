using FunnyRouletteConsole.Physics;
using FunnyRouletteConsole.Rendering;

namespace FunnyRouletteConsole.Core
{
    /// <summary>
    /// Represents the ball in the roulette simulation.
    /// </summary>
    public class Ball : IPhysicsObject, IRenderable
    {
        // Angular position of the ball in degrees
        public double Position { get; set; }
        public double AngularVelocity { get; set; }

        public double InitialAngularVelocity { get; set; }
        public double DecayCoefficient { get; set; } = 0.3; // Adjusted for realistic spin duration
        public double StoppingThreshold { get; set; } = 5.0;
        public double DropThreshold { get; set; } = 100.0;
        public bool IsStopped { get; private set; } = false;

        // Physical properties
        public double Mass { get; set; } = 0.05; // In kilograms
        public double Radius { get; set; } = 0.02; // Ball radius (normalized units)
        public double BallRadius { get; set; } = 0.9; // Radius of ball's path (normalized units)
        public double MomentOfInertia => 0.4 * Mass * Radius * Radius; // Solid sphere

        private RouletteWheel _wheel;
        private double elapsedTime = 0;
        private bool hasDropped = false;

        /// <summary>
        /// Initializes a new instance of the Ball class.
        /// </summary>
        public Ball(RouletteWheel wheel)
        {
            _wheel = wheel;
        }

        /// <summary>
        /// Updates the ball's position and velocity.
        /// </summary>
        public void Update(double deltaTime)
        {
            if (IsStopped)
                return;

            elapsedTime += deltaTime;

            // Update angular velocity using exponential decay
            AngularVelocity = InitialAngularVelocity * Math.Exp(-DecayCoefficient * elapsedTime);

            // Simulate ball dropping into the pockets
            if (!hasDropped && AngularVelocity <= DropThreshold)
            {
                hasDropped = true;
                Console.WriteLine("[Ball] The ball is dropping into the pockets.");
            }

            // Update position
            Position += AngularVelocity * deltaTime;
            Position %= 360;

            // Collision detection
            if (hasDropped)
            {
                CheckCollisions(deltaTime);
                ApplyRollingResistance(deltaTime);
            }

            // Check stopping condition
            if (AngularVelocity <= StoppingThreshold)
            {
                AngularVelocity = 0;
                IsStopped = true;
                Console.WriteLine("[Ball] The ball has stopped.");
            }
        }

        private void CheckCollisions(double deltaTime)
        {
            foreach (var fret in _wheel.Frets)
            {
                var (fretStart, fretEnd) = fret.GetLinePoints(_wheel.Position);

                // Ball position in Cartesian coordinates
                double angleRad = Position * Math.PI / 180.0;
                double ballX = BallRadius * Math.Cos(angleRad);
                double ballY = BallRadius * Math.Sin(angleRad);
                var ballPosition = new Vector2D(ballX, ballY);

                // Enhanced collision detection using Circle-Line intersection
                if (CircleLineCollision(ballPosition, Radius, fretStart, fretEnd, out Vector2D collisionPoint))
                {
                    HandleCollision(fret, collisionPoint, deltaTime);
                    Console.WriteLine($"[Ball] Collision with fret at angle: {fret.Angle:F2}°");
                    break;
                }
            }
        }

        private bool CircleLineCollision(Vector2D circleCenter, double circleRadius, Vector2D lineStart, Vector2D lineEnd, out Vector2D collisionPoint)
        {
            // Get the line segment's direction vector
            Vector2D lineDirection = lineEnd - lineStart;
            double lineLength = lineDirection.Length();
            lineDirection = lineDirection.Normalize();

            // Vector from lineStart to circle center
            Vector2D startToCircle = circleCenter - lineStart;
            double projectionLength = startToCircle.Dot(lineDirection);

            // Clamp the projection length to the length of the line segment
            projectionLength = Math.Clamp(projectionLength, 0, lineLength);
            collisionPoint = lineStart + lineDirection * projectionLength;

            // Calculate distance from circle center to the projection point (collision point)
            double distanceToLine = (circleCenter - collisionPoint).Length();

            // Check if the distance is less than or equal to the circle's radius
            return distanceToLine <= circleRadius;
        }

        private void HandleCollision(Fret fret, Vector2D collisionPoint, double deltaTime)
        {
            // Calculate normal vector at the collision point
            Vector2D normal = (PositionToCartesian() - collisionPoint).Normalize();

            // Convert angular velocity to linear velocity
            double ballSpeed = AngularVelocity * (Math.PI / 180.0) * BallRadius; // Convert deg/s to units/s
            Vector2D velocity = VelocityFromAngle(Position, ballSpeed);

            // Decompose velocity into normal and tangential components
            double vNormal = velocity.Dot(normal);
            Vector2D vNormalVec = normal * vNormal;
            Vector2D vTangentVec = velocity - vNormalVec;

            // Apply coefficient of restitution for normal component
            double restitutionCoefficient = 0.4; // Adjusted for a more elastic collision
            vNormalVec = vNormalVec * (-restitutionCoefficient);

            // Apply friction coefficient for tangential component
            double slidingFrictionCoefficient = 0.2; // Higher friction for sliding
            vTangentVec = vTangentVec * (1 - slidingFrictionCoefficient);

            // Calculate new velocity
            Vector2D newVelocity = vNormalVec + vTangentVec;

            // Update angular velocity based on new speed
            double newSpeed = newVelocity.Length();
            AngularVelocity = (newSpeed / BallRadius) * (180.0 / Math.PI); // Convert back to deg/s

            // Apply torque to the wheel based on impact force
            ApplyTorqueToWheel(vNormalVec, collisionPoint);

            // Update position and velocity
            double newAngleRad = Math.Atan2(newVelocity.Y, newVelocity.X);
            Position = (newAngleRad * 180.0 / Math.PI + 360) % 360;
        }

        private void ApplyTorqueToWheel(Vector2D collisionForce, Vector2D collisionPoint)
        {
            // Calculate torque based on collision force and point relative to wheel center
            Vector2D forceDirection = collisionPoint - _wheel.GetCenter();
            double torque = forceDirection.Cross(collisionForce); // 2D cross product

            // Apply torque to wheel's angular velocity change
            double angularAcceleration = torque / _wheel.MomentOfInertia;
            _wheel.AngularVelocity += angularAcceleration;
        }

        private void ApplyRollingResistance(double deltaTime)
        {
            double rollingResistanceCoefficient = 0.01; // Adjust as needed
            double rollingResistanceTorque = rollingResistanceCoefficient * BallRadius * Mass * 9.81; // Mass * g
            double angularDeceleration = rollingResistanceTorque / MomentOfInertia;
            AngularVelocity -= angularDeceleration * deltaTime;
            if (AngularVelocity < 0)
                AngularVelocity = 0;
        }

        private Vector2D PositionToCartesian()
        {
            double angleRad = Position * Math.PI / 180.0;
            double x = BallRadius * Math.Cos(angleRad);
            double y = BallRadius * Math.Sin(angleRad);
            return new Vector2D(x, y);
        }

        private Vector2D VelocityFromAngle(double angleDegrees, double speed)
        {
            double angleRad = angleDegrees * Math.PI / 180.0;
            double vx = -speed * Math.Sin(angleRad);
            double vy = speed * Math.Cos(angleRad);
            return new Vector2D(vx, vy);
        }

        public void Draw()
        {
            // Simplified output
            Console.WriteLine($"Ball Position: {Position:F2}°, Angular Velocity: {AngularVelocity:F2}°/s");
        }
    }
}