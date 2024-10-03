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
            PhysicsLogger.Log("--- Rendering Frame ---", PhysicsLogger.LogLevel.Debug);
            _wheel.Draw();
            _ball.Draw();
            PhysicsLogger.Log("----------------------", PhysicsLogger.LogLevel.Debug);
        }
    }
}