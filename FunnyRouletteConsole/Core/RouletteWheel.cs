using FunnyRouletteConsole.Physics;
using FunnyRouletteConsole.Rendering;

namespace FunnyRouletteConsole.Core
{
    /// <summary>
    /// Represents the roulette wheel in the simulation.
    /// </summary>
    public class RouletteWheel : IPhysicsObject, IRenderable
    {
        public double Position { get; set; }
        public double AngularVelocity { get; set; }

        public double InitialAngularVelocity { get; set; }
        public double DecayCoefficient { get; set; } = 0.07;
        public double StoppingThreshold { get; set; } = 1.0;
        public bool IsStopped { get; private set; } = false;

        public double Mass { get; set; } = 10.0;
        public double WheelRadius { get; set; } = 1.0;
        public double MomentOfInertia => 0.5 * Mass * WheelRadius * WheelRadius;

        public int NumberOfSlots { get; set; } = 37; // Standard European roulette
        public List<double> SlotPositions { get; private set; }
        public List<Fret> Frets { get; private set; }

        private double elapsedTime = 0;

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

        public void Update(double deltaTime)
        {
            if (IsStopped) return;

            elapsedTime += deltaTime;
            AngularVelocity = InitialAngularVelocity * Math.Exp(-DecayCoefficient * elapsedTime);

            if (AngularVelocity <= StoppingThreshold)
            {
                AngularVelocity = 0;
                IsStopped = true;
                PhysicsLogger.Log("[Wheel] The wheel has stopped.", PhysicsLogger.LogLevel.Info);
            }

            Position += AngularVelocity * deltaTime;
            Position %= 360;
        }

        public void Draw()
        {
            PhysicsLogger.Log($"Wheel Position: {Position:F2}°, Angular Velocity: {AngularVelocity:F2}°/s", PhysicsLogger.LogLevel.Debug);
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
    }
}