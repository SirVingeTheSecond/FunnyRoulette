using FunnyRouletteCore.Core;
using FunnyRouletteCore.Physics;
using FunnyRouletteCore.Rendering;
using System;

namespace FunnyRouletteWPF
{
    public enum SimulationState
    {
        Idle,
        Spinning,
        Ended
    }

    public class RouletteSimulation
    {
        private RouletteWheel _wheel;
        private Ball _ball;
        private PhysicsEngine _physicsEngine;
        private Renderer _renderer;
        public SimulationState CurrentState { get; private set; }

        private const double MinSimulationTime = 7.0; // Minimum simulation time in seconds
        private const double MaxSimulationTime = 15.0; // Maximum simulation time in seconds
        private double _elapsedTime;

        public RouletteSimulation()
        {
            InitializeSimulation();
        }

        private void InitializeSimulation()
        {
            _wheel = new RouletteWheel
            {
                Position = 0,
                AngularVelocity = 30,
                DecayCoefficient = 0.3, // Increased to slow down faster
            };

            _ball = new Ball(_wheel)
            {
                Position = 0,
                AngularVelocity = 180,
                DecayCoefficient = 0.5, // Increased to slow down faster
            };

            _physicsEngine = new PhysicsEngine();
            _physicsEngine.AddObject(_wheel);
            _physicsEngine.AddObject(_ball);

            _renderer = new Renderer(_wheel, _ball);

            CurrentState = SimulationState.Idle;
            _elapsedTime = 0;
        }

        public void Update(double deltaTime)
        {
            switch (CurrentState)
            {
                case SimulationState.Idle:
                    UpdateIdle(deltaTime);
                    break;
                case SimulationState.Spinning:
                    UpdateSpinning(deltaTime);
                    break;
                case SimulationState.Ended:
                    // Do nothing, waiting for reset
                    break;
            }

            _renderer.Render();
        }

        private void UpdateIdle(double deltaTime)
        {
            // Continuous rotation for idle state
            _wheel.UpdateIdle(deltaTime);
            _ball.UpdateIdle(deltaTime);
        }

        private void UpdateSpinning(double deltaTime)
        {
            _elapsedTime += deltaTime;

            _physicsEngine.Update(deltaTime);

            if (_elapsedTime >= MinSimulationTime && _wheel.AngularVelocity < 1.0)
            {
                _wheel.Stop();
                _ball.Stop();
                CurrentState = SimulationState.Ended;
                PhysicsLogger.Log($"Simulation ended after {_elapsedTime:F2} seconds", PhysicsLogger.LogLevel.Info);
            }
            else if (_elapsedTime >= MaxSimulationTime)
            {
                _wheel.Stop();
                _ball.Stop();
                CurrentState = SimulationState.Ended;
                PhysicsLogger.Log($"Simulation forced to end after maximum time of {MaxSimulationTime} seconds", PhysicsLogger.LogLevel.Info);
            }
        }

        public void StartSpin()
        {
            if (CurrentState == SimulationState.Idle)
            {
                CurrentState = SimulationState.Spinning;
                _wheel.StartSpin();
                _ball.StartSpin();
                _elapsedTime = 0;
            }
        }

        public void Reset()
        {
            if (CurrentState == SimulationState.Ended)
            {
                CurrentState = SimulationState.Idle;
                _wheel.Reset();
                _ball.Reset();
                _elapsedTime = 0;
            }
        }

        public (double WheelPosition, double BallX, double BallY) GetState()
        {
            var (ballX, ballY) = _ball.GetPosition();
            return (_wheel.Position, ballX, ballY);
        }
    }
}