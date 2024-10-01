using FunnyRouletteConsole.Physics;
using FunnyRouletteConsole.Rendering;

namespace FunnyRouletteConsole.Core
{
    /// <summary>
    /// Represents the roulette wheel in the simulation.
    /// </summary>
    public class RouletteWheel : IPhysicsObject, IRenderable
    {
        // Angular position of the wheel in degrees
        public double Position { get; set; }
        public double AngularVelocity { get; set; }

        public double InitialAngularVelocity { get; set; }
        public double DecayCoefficient { get; set; } = 0.07; // Adjusted for realistic spin duration
        public double StoppingThreshold { get; set; } = 1.0;
        public bool IsStopped { get; private set; } = false;

        // Physical properties
        public double Mass { get; set; } = 10.0; // In kilograms
        public double WheelRadius { get; set; } = 1.0; // Normalized radius
        public double MomentOfInertia => 0.5 * Mass * WheelRadius * WheelRadius; // Solid disk

        // Slots on the wheel
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

        /// <summary>
        /// Updates the wheel's position and velocity.
        /// </summary>
        public void Update(double deltaTime)
        {
            if (IsStopped)
                return;

            elapsedTime += deltaTime;

            // Update angular velocity using exponential decay
            AngularVelocity = InitialAngularVelocity * Math.Exp(-DecayCoefficient * elapsedTime);

            // Check stopping condition
            if (AngularVelocity <= StoppingThreshold)
            {
                AngularVelocity = 0;
                IsStopped = true;
            }

            // Update position
            Position += AngularVelocity * deltaTime;
            Position %= 360;
        }

        public void Draw()
        {
            // Simplified output
            Console.WriteLine($"Wheel Position: {Position:F2}°, Angular Velocity: {AngularVelocity:F2}°/s");
        }

        /// <summary>
        /// Determines the slot at the given position.
        /// </summary>
        public int GetSlotAtPosition(double ballPosition)
        {
            // Adjust ball position relative to wheel rotation
            double relativePosition = (ballPosition - Position + 360) % 360;
            double anglePerSlot = 360.0 / NumberOfSlots;
            int slotIndex = (int)(relativePosition / anglePerSlot);
            return slotIndex;
        }
    }
}