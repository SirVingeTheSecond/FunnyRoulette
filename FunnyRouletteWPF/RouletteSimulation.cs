using FunnyRouletteCore.Core;
using FunnyRouletteCore.Physics;
using FunnyRouletteCore.Rendering;

namespace FunnyRouletteWPF
{
    public class RouletteSimulation
    {
        private RouletteWheel _wheel;
        private Ball _ball;
        private PhysicsEngine _physicsEngine;
        private Renderer _renderer;

        public RouletteSimulation()
        {
            InitializeSimulation();
        }

        private void InitializeSimulation()
        {
            _wheel = new RouletteWheel
            {
                Position = 0,
                InitialAngularVelocity = 30,
                AngularVelocity = 30,
                DecayCoefficient = 0.05,
            };

            _ball = new Ball(_wheel)
            {
                Position = 0,
                InitialAngularVelocity = 180,
                AngularVelocity = 180,
                DecayCoefficient = 0.3,
            };

            _physicsEngine = new PhysicsEngine();
            _physicsEngine.AddObject(_wheel);
            _physicsEngine.AddObject(_ball);

            _renderer = new Renderer(_wheel, _ball);
        }

        public void Update(double deltaTime)
        {
            _physicsEngine.Update(deltaTime);
            _renderer.Render();
        }

        public (double WheelPosition, double BallX, double BallY) GetState()
        {
            var (ballX, ballY) = _ball.GetPosition();
            return (_wheel.Position, ballX, ballY);
        }
    }
}