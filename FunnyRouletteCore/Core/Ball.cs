using System;
using FunnyRouletteCore.Physics;
using FunnyRouletteCore.Rendering;

namespace FunnyRouletteCore.Core
{
    public class Ball : IPhysicsObject, IRenderable
    {
        public double Position { get; set; } // Changed to settable
        public double AngularVelocity { get; set; } // Changed to settable

        public double InitialAngularVelocity { get; set; }
        public double DecayCoefficient { get; set; } = 0.5;
        public double StoppingThreshold { get; set; } = 1.0;
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
        private bool isTransitioningToPocket = false;
        private int pocketIndex = -1;
        private double relativePositionInPocket;

        private const double IdleAngularVelocity = 180;
        private const double SpinAngularVelocity = 720;
        private bool _isSpinning = false;

        private const double Gravity = 9.81;
        private const double FrictionCoefficient = 0.3;
        private Vector2D velocity;

        private double transitionProgress = 0;
        private Vector2D startTransitionPosition;
        private Vector2D endTransitionPosition;

        public Ball(RouletteWheel wheel)
        {
            _wheel = wheel;
            BallRadius = OuterRadius;
        }

        public void UpdateIdle(double deltaTime)
        {
            Position = (Position + IdleAngularVelocity * deltaTime) % 360;
        }

        public void StartSpin()
        {
            _isSpinning = true;
            InitialAngularVelocity = SpinAngularVelocity;
            AngularVelocity = SpinAngularVelocity;
            elapsedTime = 0;
            IsStopped = false;
            hasDropped = false;
            isSettledInPocket = false;
            isTransitioningToPocket = false;
            BallRadius = OuterRadius;
            velocity = new Vector2D(0, -AngularVelocity * Math.PI / 180 * BallRadius);
        }

        public void Reset()
        {
            _isSpinning = false;
            AngularVelocity = IdleAngularVelocity;
            IsStopped = false;
            hasDropped = false;
            isSettledInPocket = false;
            isTransitioningToPocket = false;
            BallRadius = OuterRadius;
        }

        public void Update(double deltaTime)
        {
            if (!_isSpinning || IsStopped)
            {
                return;
            }

            elapsedTime += deltaTime;

            if (isSettledInPocket)
            {
                UpdateSettledBallPosition();
            }
            else if (isTransitioningToPocket)
            {
                UpdateTransitioningBall(deltaTime);
            }
            else
            {
                UpdateSpinningBall(deltaTime);
            }
        }

        private void UpdateSpinningBall(double deltaTime)
        {
            AngularVelocity = InitialAngularVelocity * Math.Exp(-DecayCoefficient * elapsedTime);
            
            Position = (Position + AngularVelocity * deltaTime) % 360;

            // Check if the ball should transition to a pocket
            if (AngularVelocity <= DropThreshold && !hasDropped)
            {
                StartTransitionToPocket();
            }

            CheckCollisions(deltaTime);
            ApplyRollingResistance(deltaTime);
        }

        private void StartTransitionToPocket()
        {
            hasDropped = true;
            isTransitioningToPocket = true;
            transitionProgress = 0;

            // Calculate start and end positions for the transition
            startTransitionPosition = PositionToCartesian();
            pocketIndex = _wheel.GetSlotAtPosition(Position);
            double pocketAngle = GetPocketAngle(pocketIndex);
            endTransitionPosition = new Vector2D(
                InnerRadius * Math.Cos(pocketAngle * Math.PI / 180),
                InnerRadius * Math.Sin(pocketAngle * Math.PI / 180)
            );

            PhysicsLogger.Log("[Ball] The ball is transitioning to a pocket.", PhysicsLogger.LogLevel.Info);
        }

        private void UpdateTransitioningBall(double deltaTime)
        {
            // Progress the transition
            transitionProgress += deltaTime * 2; // Adjust this multiplier to control transition speed
            transitionProgress = Math.Min(transitionProgress, 1);

            // Use easing function for smooth transition
            double easedProgress = EaseInOutCubic(transitionProgress);

            // Interpolate between start and end positions
            Vector2D currentPosition = Vector2D.Lerp(startTransitionPosition, endTransitionPosition, easedProgress);

            // Update ball properties
            Position = (Math.Atan2(currentPosition.Y, currentPosition.X) * 180 / Math.PI + 360) % 360;
            BallRadius = OuterRadius + (InnerRadius - OuterRadius) * easedProgress;

            // Check if transition is complete
            if (transitionProgress >= 1)
            {
                SettleIntoPocket();
            }
        }

        private double EaseInOutCubic(double t)
        {
            return t < 0.5 ? 4 * t * t * t : 1 - Math.Pow(-2 * t + 2, 3) / 2;
        }

        private void SettleIntoPocket()
        {
            isSettledInPocket = true;
            isTransitioningToPocket = false;
            BallRadius = InnerRadius;
            relativePositionInPocket = (Position - _wheel.Position + 360) % 360;
            AngularVelocity = _wheel.AngularVelocity;
            PhysicsLogger.Log($"[Ball] The ball has settled into pocket {pocketIndex}.", PhysicsLogger.LogLevel.Info);
        }

        private void UpdateSettledBallPosition()
        {
            // Follow the wheel's rotation
            Position = (_wheel.Position + relativePositionInPocket) % 360;
            AngularVelocity = _wheel.AngularVelocity;
        }

        private double GetPocketAngle(int pocketIndex)
        {
            double anglePerPocket = 360.0 / _wheel.NumberOfSlots;
            return pocketIndex * anglePerPocket;
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

            double restitutionCoefficient = 0.3;
            vNormalVec = vNormalVec * (-restitutionCoefficient);

            double slidingFrictionCoefficient = 0.1;
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
            double rollingResistanceTorque = rollingResistanceCoefficient * BallRadius * Mass * Gravity;
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

        public void Stop()
        {
            IsStopped = true;
            AngularVelocity = 0;
            velocity = new Vector2D(0, 0);
            PhysicsLogger.Log("[Ball] The ball has stopped.", PhysicsLogger.LogLevel.Info);
        }

        public void Draw()
        {
            string status = isSettledInPocket ? $"Settled in pocket {pocketIndex}" : 
                            (isTransitioningToPocket ? $"Transitioning to pocket (Progress: {transitionProgress:P0})" : 
                            (IsStopped ? "Stopped" : "Moving"));
            PhysicsLogger.Log($"Ball Position: {Position:F2}°, Angular Velocity: {AngularVelocity:F2}°/s, Radius: {BallRadius:F3}, Status: {status}", PhysicsLogger.LogLevel.Debug);
        }
    }
}