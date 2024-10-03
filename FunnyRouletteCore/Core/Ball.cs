using FunnyRouletteCore.Physics;
using FunnyRouletteCore.Rendering;

namespace FunnyRouletteCore.Core
{
    /// <summary>
    /// Represents the ball in the roulette simulation.
    /// </summary>
    public class Ball : IPhysicsObject, IRenderable
    {
        public double Position { get; set; } // Angular position of the ball in degrees
        public double AngularVelocity { get; set; } // Angular velocity of the ball in degrees/second

        public double InitialAngularVelocity { get; set; }
        public double DecayCoefficient { get; set; } = 0.3;
        public double StoppingThreshold { get; set; } = 5.0;
        public double DropThreshold { get; set; } = 100.0;
        public bool IsStopped { get; private set; } = false;

        public double Mass { get; set; } = 0.05; // Mass of the ball (in kilograms)
        public double Radius { get; set; } = 0.02; // Ball radius (normalized units)
        public double BallRadius { get; set; } = 0.9; // Radius of the ball's path (normalized units)
        public double MomentOfInertia => 0.4 * Mass * Radius * Radius;

        private RouletteWheel _wheel;
        private double elapsedTime = 0;
        private bool hasDropped = false;

        public Ball(RouletteWheel wheel)
        {
            _wheel = wheel;
        }

        public void Update(double deltaTime)
        {
            if (IsStopped) return;

            elapsedTime += deltaTime;
            AngularVelocity = InitialAngularVelocity * Math.Exp(-DecayCoefficient * elapsedTime);

            if (!hasDropped && AngularVelocity <= DropThreshold)
            {
                hasDropped = true;
                PhysicsLogger.Log("[Ball] The ball is dropping into the pockets.", PhysicsLogger.LogLevel.Info);
            }

            Position += AngularVelocity * deltaTime;
            Position %= 360;

            if (hasDropped)
            {
                CheckCollisions(deltaTime);
                ApplyRollingResistance(deltaTime);
            }

            if (AngularVelocity <= StoppingThreshold)
            {
                AngularVelocity = 0;
                IsStopped = true;
                PhysicsLogger.Log("[Ball] The ball has stopped.", PhysicsLogger.LogLevel.Info);
            }
        }

        private void CheckCollisions(double deltaTime)
        {
            foreach (var fret in _wheel.Frets)
            {
                var (fretStart, fretEnd) = fret.GetLinePoints(_wheel.Position);

                double angleRad = Position * Math.PI / 180.0;
                double ballX = BallRadius * Math.Cos(angleRad);
                double ballY = BallRadius * Math.Sin(angleRad);
                var ballPosition = new Vector2D(ballX, ballY);

                if (IsPotentialCollision(ballPosition, fret) && CircleLineCollision(ballPosition, Radius, fretStart, fretEnd, out Vector2D collisionPoint))
                {
                    HandleCollision(fret, collisionPoint, deltaTime);
                    PhysicsLogger.Log($"[Ball] Collision with fret at angle: {fret.Angle:F2}°", PhysicsLogger.LogLevel.Debug);
                    break;
                }
            }
        }

        private bool IsPotentialCollision(Vector2D ballPosition, Fret fret)
        {
            var (fretStart, fretEnd) = fret.GetLinePoints(_wheel.Position);
            Vector2D fretMidpoint = (fretStart + fretEnd) * 0.5;

            double fretBoundingRadius = (fretEnd - fretMidpoint).Length() + Radius;
            double distanceToFret = (ballPosition - fretMidpoint).Length();

            return distanceToFret <= fretBoundingRadius;
        }

        private bool CircleLineCollision(Vector2D circleCenter, double circleRadius, Vector2D lineStart, Vector2D lineEnd, out Vector2D collisionPoint)
        {
            Vector2D lineDirection = lineEnd - lineStart;
            double lineLength = lineDirection.Length();
            lineDirection = lineDirection.Normalize();

            Vector2D startToCircle = circleCenter - lineStart;
            double projectionLength = startToCircle.Dot(lineDirection);
            projectionLength = Math.Clamp(projectionLength, 0, lineLength);
            collisionPoint = lineStart + lineDirection * projectionLength;

            double distanceToLine = (circleCenter - collisionPoint).Length();
            return distanceToLine <= circleRadius;
        }

        private void HandleCollision(Fret fret, Vector2D collisionPoint, double deltaTime)
        {
            Vector2D normal = (PositionToCartesian() - collisionPoint).Normalize();
            double ballSpeed = AngularVelocity * (Math.PI / 180.0) * BallRadius;
            Vector2D velocity = VelocityFromAngle(Position, ballSpeed);

            double vNormal = velocity.Dot(normal);
            Vector2D vNormalVec = normal * vNormal;
            Vector2D vTangentVec = velocity - vNormalVec;

            double restitutionCoefficient = 0.4;
            vNormalVec = vNormalVec * (-restitutionCoefficient);

            double frictionCoefficient = GetDynamicFrictionCoefficient(velocity.Length());
            vTangentVec = vTangentVec * (1 - frictionCoefficient);

            Vector2D newVelocity = vNormalVec + vTangentVec;
            double newSpeed = newVelocity.Length();
            AngularVelocity = (newSpeed / BallRadius) * (180.0 / Math.PI);

            ApplyAdvancedTorqueToWheel(vNormalVec, collisionPoint);

            double newAngleRad = Math.Atan2(newVelocity.Y, newVelocity.X);
            Position = (newAngleRad * 180.0 / Math.PI + 360) % 360;
        }

        private void ApplyAdvancedTorqueToWheel(Vector2D collisionForce, Vector2D collisionPoint)
        {
            Vector2D forceDirection = collisionPoint - _wheel.GetCenter();
            double torque = forceDirection.Cross(collisionForce);

            double angularAcceleration = torque / _wheel.MomentOfInertia;
            _wheel.AngularVelocity += angularAcceleration;
        }

        private void ApplyRollingResistance(double deltaTime)
        {
            double rollingResistanceCoefficient = 0.01;
            double rollingResistanceTorque = rollingResistanceCoefficient * BallRadius * Mass * 9.81;
            double angularDeceleration = rollingResistanceTorque / MomentOfInertia;
            AngularVelocity -= angularDeceleration * deltaTime;
            if (AngularVelocity < 0) AngularVelocity = 0;
        }

        private double GetDynamicFrictionCoefficient(double velocity)
        {
            return Math.Max(0.05, 0.3 - (0.02 * velocity));
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
            PhysicsLogger.Log($"Ball Position: {Position:F2}°, Angular Velocity: {AngularVelocity:F2}°/s", PhysicsLogger.LogLevel.Debug);
        }
    }
}