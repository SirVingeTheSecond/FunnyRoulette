using System;
using FunnyRouletteCore.Physics;
using FunnyRouletteCore.Rendering;

namespace FunnyRouletteCore.Core
{
    public class Ball : IPhysicsObject, IRenderable
    {
        public double Position { get; set; } // Changed to public set
        public double AngularVelocity { get; set; } // Changed to public set

        public double InitialAngularVelocity { get; set; }
        public double DecayCoefficient { get; set; } = 0.3;
        public double StoppingThreshold { get; set; } = 5.0;
        public double DropThreshold { get; set; } = 15.0;
        public bool IsStopped { get; private set; } = false;

        public double Mass { get; set; } = 0.05;
        public double Radius { get; set; } = 0.02;
        public double OuterRadius { get; set; } = 1.0;
        public double InnerRadius { get; set; } = 0.9;
        public double BallRadius { get; private set; }
        public double MomentOfInertia => 0.4 * Mass * Radius * Radius;

        private RouletteWheel _wheel;
        private double elapsedTime = 0;
        private bool hasDropped = false;
        private bool isSettledInPocket = false;
        private int pocketIndex = -1;
        private double relativePositionInPocket;

        public Ball(RouletteWheel wheel)
        {
            _wheel = wheel;
            BallRadius = OuterRadius;
        }

        public void Update(double deltaTime)
        {
            if (IsStopped && !isSettledInPocket) return;

            elapsedTime += deltaTime;

            if (!isSettledInPocket)
            {
                AngularVelocity = InitialAngularVelocity * Math.Exp(-DecayCoefficient * elapsedTime);
                Position = (Position + AngularVelocity * deltaTime) % 360;

                if (!hasDropped && AngularVelocity <= DropThreshold)
                {
                    hasDropped = true;
                    BallRadius = InnerRadius;
                    PhysicsLogger.Log("[Ball] The ball is dropping into the pockets.", PhysicsLogger.LogLevel.Info);
                }

                if (hasDropped)
                {
                    CheckCollisions(deltaTime);
                    ApplyRollingResistance(deltaTime);
                }

                if (AngularVelocity <= StoppingThreshold && hasDropped)
                {
                    SettleIntoPocket();
                }
            }
            else
            {
                UpdateSettledBallPosition();
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

                if (CircleLineCollision(ballPosition, Radius, fretStart, fretEnd, out Vector2D collisionPoint))
                {
                    HandleCollision(fret, collisionPoint, deltaTime);
                    PhysicsLogger.Log($"[Ball] Collision with fret at angle: {fret.Angle:F2}°", PhysicsLogger.LogLevel.Debug);
                    break;
                }
            }
        }

        private void SettleIntoPocket()
        {
            isSettledInPocket = true;
            pocketIndex = _wheel.GetSlotAtPosition(Position);
            double pocketCenterAngle = GetPocketAngle(pocketIndex);
            relativePositionInPocket = (Position - _wheel.Position - pocketCenterAngle + 360) % 360;
            AngularVelocity = 0;
            IsStopped = false;
            PhysicsLogger.Log($"[Ball] The ball has settled into pocket {pocketIndex}.", PhysicsLogger.LogLevel.Info);
        }

        private void UpdateSettledBallPosition()
        {
            double pocketCenterAngle = GetPocketAngle(pocketIndex);
            Position = (_wheel.Position + pocketCenterAngle + relativePositionInPocket) % 360;
        }

        private double GetPocketAngle(int pocketIndex)
        {
            double anglePerPocket = 360.0 / _wheel.NumberOfSlots;
            return pocketIndex * anglePerPocket;
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

            double distanceToLine = Vector2D.Distance(circleCenter, collisionPoint);
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

            double restitutionCoefficient = 0.1;
            vNormalVec = vNormalVec * (-restitutionCoefficient);

            double slidingFrictionCoefficient = 0.3;
            vTangentVec = vTangentVec * (1 - slidingFrictionCoefficient);

            Vector2D newVelocity = vNormalVec + vTangentVec;

            if (newVelocity.Length() > velocity.Length())
            {
                newVelocity = velocity;
            }

            double newSpeed = newVelocity.Length();
            AngularVelocity = (newSpeed / BallRadius) * (180.0 / Math.PI);

            ApplyTorqueToWheel(vNormalVec, collisionPoint);

            Position = CorrectPositionAfterCollision(Position, collisionPoint);

            PhysicsLogger.Log($"[Ball] Post-Collision Velocity: {newVelocity.Length():F2} units/s, Angular Velocity: {AngularVelocity:F2}°/s", PhysicsLogger.LogLevel.Debug);
        }
        
        private double CorrectPositionAfterCollision(double currentPosition, Vector2D collisionPoint)
        {
            Vector2D center = new Vector2D(0, 0);
            Vector2D toCollisionPoint = collisionPoint - center;
            double correctedAngle = Math.Atan2(toCollisionPoint.Y, toCollisionPoint.X) * (180.0 / Math.PI);

            correctedAngle = (correctedAngle + 360) % 360;

            if (currentPosition > correctedAngle)
            {
                correctedAngle += 2.0;
            }
            else
            {
                correctedAngle -= 2.0;
            }

            correctedAngle = (correctedAngle + 360) % 360;

            return correctedAngle;
        }

        private void ApplyTorqueToWheel(Vector2D collisionForce, Vector2D collisionPoint)
        {
            Vector2D forceDirection = collisionPoint - _wheel.GetCenter();
            double torque = forceDirection.Cross(collisionForce);

            _wheel.ApplyTorque(torque);
        }

        private void ApplyRollingResistance(double deltaTime)
        {
            double rollingResistanceCoefficient = 0.01;
            double rollingResistanceTorque = rollingResistanceCoefficient * BallRadius * Mass * 9.81;
            double angularDeceleration = rollingResistanceTorque / MomentOfInertia;
            AngularVelocity -= angularDeceleration * deltaTime;
            if (AngularVelocity < 0) AngularVelocity = 0;
        }

        public Vector2D PositionToCartesian()
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

        public (double X, double Y) GetPosition()
        {
            Vector2D cartesianPosition = PositionToCartesian();
            return (cartesianPosition.X, cartesianPosition.Y);
        }

        public void Draw()
        {
            string status = isSettledInPocket ? $"Settled in pocket {pocketIndex}" : "Moving";
            PhysicsLogger.Log($"Ball Position: {Position:F2}°, Angular Velocity: {AngularVelocity:F2}°/s, Status: {status}", PhysicsLogger.LogLevel.Debug);
        }
    }
}