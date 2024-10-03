using FunnyRouletteCore.Core;
using FunnyRouletteCore.Physics;

namespace FunnyRouletteCore.Rendering
{
    public class Renderer
    {
        private readonly RouletteWheel _wheel;
        private readonly Ball _ball;

        public Renderer(RouletteWheel wheel, Ball ball)
        {
            _wheel = wheel;
            _ball = ball;
        }

        public void Render()
        {
            Console.Clear();
            _wheel.Draw();
            _ball.Draw();

            PhysicsLogger.Log($"Ball Trajectory: {_ball.Position:F2}°, Velocity: {_ball.AngularVelocity:F2}°/s", PhysicsLogger.LogLevel.Info);
        }
    }
}