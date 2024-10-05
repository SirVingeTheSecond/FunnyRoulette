using System;
using System.Windows;
using System.Windows.Controls;
using System.Windows.Media;
using System.Windows.Shapes;
using System.Windows.Threading;

namespace FunnyRouletteWPF
{
    public partial class MainWindow : Window
    {
        private RouletteSimulation _simulation;
        private DispatcherTimer _timer;

        // UI elements
        private Ellipse _wheelEllipse;
        private Ellipse _ballEllipse;
        private Line[] _fretLines;
        private Button _actionButton;

        public MainWindow()
        {
            InitializeComponent();

            InitializeSimulation();
            SetupVisualElements();
            StartSimulationLoop();
        }

        private void InitializeSimulation()
        {
            _simulation = new RouletteSimulation();
        }

        private void SetupVisualElements()
        {
            // Create visual elements for the wheel
            _wheelEllipse = new Ellipse
            {
                Width = 400,
                Height = 400,
                Stroke = Brushes.Black,
                StrokeThickness = 2
            };
            SimulationCanvas.Children.Add(_wheelEllipse);

            // Create visual elements for the ball
            _ballEllipse = new Ellipse
            {
                Width = 10,
                Height = 10,
                Fill = Brushes.Red
            };
            SimulationCanvas.Children.Add(_ballEllipse);

            // Create visual elements for frets
            _fretLines = new Line[37]; // Assuming 37 slots for European roulette
            for (int i = 0; i < _fretLines.Length; i++)
            {
                _fretLines[i] = new Line
                {
                    Stroke = Brushes.Black,
                    StrokeThickness = 1
                };
                SimulationCanvas.Children.Add(_fretLines[i]);
            }

            // Create action button
            _actionButton = new Button
            {
                Content = "Spin",
                Width = 100,
                Height = 30,
                Margin = new Thickness(0, 10, 0, 0)
            };
            _actionButton.Click += ActionButton_Click;
            Grid.SetRow(_actionButton, 1);
            MainGrid.Children.Add(_actionButton);
        }

        private void ActionButton_Click(object sender, RoutedEventArgs e)
        {
            switch (_simulation.CurrentState)
            {
                case SimulationState.Idle:
                    _simulation.StartSpin();
                    _actionButton.Content = "Spinning...";
                    _actionButton.IsEnabled = false;
                    break;
                case SimulationState.Ended:
                    _simulation.Reset();
                    _actionButton.Content = "Spin";
                    break;
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
            _simulation.Update(deltaTime);

            // Update visual positions
            UpdateVisualPositions();

            // Update UI based on simulation state
            if (_simulation.CurrentState == SimulationState.Ended && _actionButton.Content.ToString() != "Reset")
            {
                _actionButton.Content = "Reset";
                _actionButton.IsEnabled = true;
            }
        }

        private void UpdateVisualPositions()
        {
            var (wheelPosition, ballX, ballY) = _simulation.GetState();

            // Get the center of the canvas
            double centerX = SimulationCanvas.ActualWidth / 2;
            double centerY = SimulationCanvas.ActualHeight / 2;

            // Update the wheel's rotation
            RotateTransform wheelRotation = new RotateTransform(wheelPosition, centerX, centerY);
            _wheelEllipse.RenderTransform = wheelRotation;

            // Update the ball's position
            Canvas.SetLeft(_ballEllipse, centerX + ballX * 200 - _ballEllipse.Width / 2);
            Canvas.SetTop(_ballEllipse, centerY + ballY * 200 - _ballEllipse.Height / 2);

            // Update fret lines
            for (int i = 0; i < _fretLines.Length; i++)
            {
                double angle = (i * 360.0 / _fretLines.Length + wheelPosition) * Math.PI / 180.0;
                _fretLines[i].X1 = centerX + 180 * Math.Cos(angle);
                _fretLines[i].Y1 = centerY + 180 * Math.Sin(angle);
                _fretLines[i].X2 = centerX + 200 * Math.Cos(angle);
                _fretLines[i].Y2 = centerY + 200 * Math.Sin(angle);
            }
        }
    }
}