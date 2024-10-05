using System;
using System.Collections.Generic;
using FunnyRouletteCore.Physics;
using FunnyRouletteCore.Rendering;

namespace FunnyRouletteCore.Core
{
    public class RouletteWheel : IPhysicsObject, IRenderable
    {
        public double Position { get; set; }
        public double AngularVelocity { get; set; }

        public double InitialAngularVelocity { get; set; }
        public double DecayCoefficient { get; set; } = 0.3; // Increased to slow down faster
        public double StoppingThreshold { get; set; } = 1.0;
        public bool IsStopped { get; private set; } = false;

        public double Mass { get; set; } = 10.0;
        public double WheelRadius { get; set; } = 1.0;
        public double MomentOfInertia => 0.5 * Mass * WheelRadius * WheelRadius;

        public int NumberOfSlots { get; set; } = 37; // Standard European roulette
        public List<double> SlotPositions { get; private set; }
        public List<Fret> Frets { get; private set; }

        private double elapsedTime = 0;

        private const double IdleAngularVelocity = 30;
        private const double SpinAngularVelocity = 360;
        private bool _isSpinning = false;

        public RouletteWheel()
        {
            InitializeSlots();
        }

        private void InitializeSlots()
        {
            SlotPositions = new List<double>();
            Frets = new List<Fret>();
            double anglePerSlot = 360.0 / NumberOfSlots;
            for (int i = 0; i < NumberOfSlots; i++)
            {
                double angle = i * anglePerSlot;
                SlotPositions.Add(angle);
                Frets.Add(new Fret(angle));
            }
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
        }

        public void Reset()
        {
            _isSpinning = false;
            AngularVelocity = IdleAngularVelocity;
            IsStopped = false;
        }

        public void Update(double deltaTime)
        {
            if (!_isSpinning)
            {
                return;
            }

            if (IsStopped) return;

            elapsedTime += deltaTime;
            AngularVelocity = InitialAngularVelocity * Math.Exp(-DecayCoefficient * elapsedTime);

            if (AngularVelocity <= StoppingThreshold)
            {
                Stop();
            }

            Position += AngularVelocity * deltaTime;
            Position %= 360;
        }

        public void Stop()
        {
            AngularVelocity = 0;
            IsStopped = true;
            PhysicsLogger.Log("[Wheel] The wheel has stopped.", PhysicsLogger.LogLevel.Info);
        }

        public int GetSlotAtPosition(double ballPosition)
        {
            double relativePosition = (ballPosition - Position + 360) % 360;
            double anglePerSlot = 360.0 / NumberOfSlots;
            int slotIndex = (int)(relativePosition / anglePerSlot);
            return slotIndex;
        }

        public void ApplyTorque(double torque)
        {
            double angularAcceleration = torque / MomentOfInertia;
            AngularVelocity += angularAcceleration;
        }

        public Vector2D GetCenter()
        {
            return new Vector2D(0, 0);
        }

        public void Draw()
        {
            string status = IsStopped ? "Stopped" : "Spinning";
            PhysicsLogger.Log($"Wheel Position: {Position:F2}°, Angular Velocity: {AngularVelocity:F2}°/s, Status: {status}", PhysicsLogger.LogLevel.Debug);
        }
    }
}