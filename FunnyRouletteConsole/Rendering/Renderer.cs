using FunnyRouletteConsole.Core;

namespace FunnyRouletteConsole.Rendering
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

            Console.WriteLine($"Ball Trajectory: {_ball.Position:F2}°, Velocity: {_ball.AngularVelocity:F2}°/s");
        }
    }
}