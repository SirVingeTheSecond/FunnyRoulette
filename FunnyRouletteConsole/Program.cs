using FunnyRouletteCore.Core;
using FunnyRouletteCore.Physics;
using FunnyRouletteCore.Rendering;

namespace FunnyRouletteConsole
{
    class Program
    {
        static void Main(string[] args)
        {
            // Initialize random number generator
            var random = new Random();

            // Set initial angular velocities (rad/s converted to deg/s)
            double wheelInitialAngularVelocityRad = random.NextDouble() * (4 - 2) + 2; // 2 to 4 rad/s
            double wheelInitialAngularVelocity = wheelInitialAngularVelocityRad * (180 / Math.PI); // Convert to deg/s

            double ballInitialAngularVelocityRad = random.NextDouble() * (15 - 10) + 10; // 10 to 15 rad/s
            double ballInitialAngularVelocity = ballInitialAngularVelocityRad * (180 / Math.PI); // Convert to deg/s

            // Set decay coefficients
            double wheelDecayCoefficient = random.NextDouble() * (0.1 - 0.05) + 0.05; // k between 0.05 and 0.1
            double ballDecayCoefficient = random.NextDouble() * (0.4 - 0.2) + 0.2; // μ between 0.2 and 0.4

            // Create the roulette wheel
            var rouletteWheel = new RouletteWheel
            {
                Position = 0,
                InitialAngularVelocity = wheelInitialAngularVelocity,
                AngularVelocity = wheelInitialAngularVelocity,
                DecayCoefficient = wheelDecayCoefficient,
                StoppingThreshold = 1.0 // Degrees per second
            };

            // Create the ball and pass the wheel to it
            var ball = new Ball(rouletteWheel)
            {
                Position = random.NextDouble() * 360, // Random starting position
                InitialAngularVelocity = ballInitialAngularVelocity,
                AngularVelocity = ballInitialAngularVelocity,
                DecayCoefficient = ballDecayCoefficient,
                StoppingThreshold = 5.0, // Degrees per second
                DropThreshold = 100.0 // Degrees per second when the ball drops into pockets
            };

            // Initialize Physics Engine and Renderer
            var physicsEngine = new PhysicsEngine();
            physicsEngine.AddObject(rouletteWheel);
            physicsEngine.AddObject(ball);

            var renderer = new Renderer(rouletteWheel, ball);

            double totalTime = 0;
            double deltaTime = 0.05; // 50ms per frame

            while (true)
            {
                physicsEngine.Update(deltaTime);
                renderer.Render();

                totalTime += deltaTime;

                // Check if simulation should stop
                if (ball.IsStopped)
                {
                    int winningSlot = rouletteWheel.GetSlotAtPosition(ball.Position);
                    Console.WriteLine($"\nThe ball has landed on slot: {winningSlot}");
                    break;
                }

                // Limit total simulation time to prevent infinite loops
                if (totalTime >= 30) // 30 seconds max
                {
                    Console.WriteLine("\nSimulation exceeded maximum duration and was terminated.");
                    break;
                }

                Thread.Sleep((int)(deltaTime * 1000));
            }

            Console.WriteLine($"\nSimulation complete in {totalTime:F2} seconds.");
        }
    }
}