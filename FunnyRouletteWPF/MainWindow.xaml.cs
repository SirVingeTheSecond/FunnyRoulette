using System.Windows;
using System.Windows.Controls;
using System.Windows.Media;
using System.Windows.Shapes;
using System.Windows.Threading;
using FunnyRouletteCore.Core;

namespace FunnyRouletteWPF
{
    public partial class MainWindow : Window
    {
        private RouletteWheel _wheel;
        private Ball _ball;
        private DispatcherTimer _timer;

        // UI elements
        private Ellipse _ballEllipse;
        private Line[] _fretLines;

        public MainWindow()
        {
            InitializeComponent();

            InitializeSimulation();
            SetupVisualElements();
            StartSimulationLoop();
        }

        private void InitializeSimulation()
        {
            // Initialize physics simulation objects
            _wheel = new RouletteWheel
            {
                Position = 0,
                InitialAngularVelocity = 30, // Set initial angular velocity
                AngularVelocity = 30,
                DecayCoefficient = 0.05, // Friction/decay coefficient for the wheel
            };

            _ball = new Ball(_wheel)
            {
                Position = 0,
                InitialAngularVelocity = 180, // Set initial angular velocity of the ball
                AngularVelocity = 180,
                DecayCoefficient = 0.3, // Friction/decay coefficient for the ball
            };
        }

        private void SetupVisualElements()
        {
            // Create visual elements for the ball and frets
            _ballEllipse = new Ellipse
            {
                Width = 10,
                Height = 10,
                Fill = Brushes.Red
            };
            SimulationCanvas.Children.Add(_ballEllipse);

            // Create visual elements for frets
            _fretLines = new Line[_wheel.NumberOfSlots];
            for (int i = 0; i < _wheel.NumberOfSlots; i++)
            {
                _fretLines[i] = new Line
                {
                    Stroke = Brushes.Black,
                    StrokeThickness = 2
                };
                SimulationCanvas.Children.Add(_fretLines[i]);
            }
        }

        private void StartSimulationLoop()
        {
            _timer = new DispatcherTimer
            {
                Interval = TimeSpan.FromMilliseconds(16) // Approx 60 FPS
            };
            _timer.Tick += UpdateSimulation;
            _timer.Start();
        }

        private void UpdateSimulation(object sender, EventArgs e)
        {
            // Update physics state
            double deltaTime = 0.016; // Fixed deltaTime for simplicity (16 ms)
            _ball.Update(deltaTime);
            _wheel.Update(deltaTime);

            // Update visual positions
            UpdateVisualPositions();
        }

        private void UpdateVisualPositions()
        {
            // Get the center of the canvas
            double centerX = SimulationCanvas.ActualWidth / 2;
            double centerY = SimulationCanvas.ActualHeight / 2;

            // Update the ball's position based on its current angle
            Vector2D ballPosition = _ball.PositionToCartesian();
            Canvas.SetLeft(_ballEllipse, centerX + ballPosition.X * 100 - _ballEllipse.Width / 2);
            Canvas.SetTop(_ballEllipse, centerY + ballPosition.Y * 100 - _ballEllipse.Height / 2);

            // Update fret lines based on their positions
            for (int i = 0; i < _wheel.Frets.Count; i++)
            {
                var (start, end) = _wheel.Frets[i].GetLinePoints(_wheel.Position);

                _fretLines[i].X1 = centerX + start.X * 100;
                _fretLines[i].Y1 = centerY + start.Y * 100;
                _fretLines[i].X2 = centerX + end.X * 100;
                _fretLines[i].Y2 = centerY + end.Y * 100;
            }
        }
    }
}