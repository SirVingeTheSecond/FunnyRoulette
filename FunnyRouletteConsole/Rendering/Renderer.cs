using FunnyRouletteConsole.Core;

namespace FunnyRouletteConsole.Rendering
{
    /// <summary>
    /// Responsible for rendering the simulation.
    /// </summary>
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
        }
    }
}