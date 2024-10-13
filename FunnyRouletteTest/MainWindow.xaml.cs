using System.Windows;
using System.Windows.Controls;
using System.Windows.Media;
using System.Windows.Shapes;

namespace FunnyRouletteWPF
{
    public partial class MainWindow : Window
    {
        private const int SectorCount = 37; // European roulette
        private const double WheelRadius = 200;
        private const double BallRadius = 10;
        private List<int> Numbers;

        public MainWindow()
        {
            InitializeComponent();

            // Initialize numbers
            Numbers = new List<int> { 0, 32, 15, 19, 4, 21, 2, 25, 17, 34, 6, 27, 13, 36, 11, 30, 8, 23, 10, 5, 24, 16, 33, 1, 20, 14, 31, 9, 22, 18, 29, 7, 28, 12, 35, 3, 26 };

            // Draw the wheel
            DrawWheel();

            // Hook up event handlers
            SpinButton.Click += SpinButton_Click;
            ResetButton.Click += ResetButton_Click;
        }

        private void DrawWheel()
        {
            double anglePerSector = 360.0 / SectorCount;
            double currentAngle = -90;

            WheelCanvas.Width = WheelRadius * 2;
            WheelCanvas.Height = WheelRadius * 2;
            WheelCanvas.RenderTransformOrigin = new Point(0.5, 0.5);

            BallCanvas.Width = WheelRadius * 2;
            BallCanvas.Height = WheelRadius * 2;
            BallCanvas.RenderTransformOrigin = new Point(0.5, 0.5);

            for (int i = 0; i < SectorCount; i++)
            {
                Path sector = CreateSector(currentAngle, anglePerSector, WheelRadius, GetSectorColor(Numbers[i]));
                WheelCanvas.Children.Add(sector);

                // Add number labels
                TextBlock label = CreateSectorLabel(currentAngle + anglePerSector / 2, WheelRadius - 20, Numbers[i].ToString());
                WheelCanvas.Children.Add(label);

                currentAngle += anglePerSector;
            }
        }

        private Path CreateSector(double startAngle, double angle, double radius, Brush fill)
        {
            double centerX = WheelRadius;
            double centerY = WheelRadius;

            Point center = new Point(centerX, centerY);
            Point startPoint = new Point(
                centerX + radius * Math.Cos(startAngle * Math.PI / 180),
                centerY + radius * Math.Sin(startAngle * Math.PI / 180));

            Point endPoint = new Point(
                centerX + radius * Math.Cos((startAngle + angle) * Math.PI / 180),
                centerY + radius * Math.Sin((startAngle + angle) * Math.PI / 180));

            PathFigure figure = new PathFigure();
            figure.StartPoint = center;
            figure.Segments.Add(new LineSegment(startPoint, true));
            figure.Segments.Add(new ArcSegment(endPoint, new Size(radius, radius), angle, angle >= 180, SweepDirection.Clockwise, true));
            figure.IsClosed = true;

            PathGeometry geometry = new PathGeometry();
            geometry.Figures.Add(figure);

            Path path = new Path();
            path.Data = geometry;
            path.Fill = fill;
            return path;
        }

        private TextBlock CreateSectorLabel(double angle, double distance, string text)
        {
            double centerX = WheelRadius;
            double centerY = WheelRadius;

            double radian = angle * Math.PI / 180;
            Point position = new Point(
                centerX + distance * Math.Cos(radian),
                centerY + distance * Math.Sin(radian));

            TextBlock label = new TextBlock();
            label.Text = text;
            label.Foreground = Brushes.White;
            label.FontSize = 14;
            Canvas.SetLeft(label, position.X - 10);
            Canvas.SetTop(label, position.Y - 10);
            return label;
        }

        private Brush GetSectorColor(int number)
        {
            if (number == 0)
            {
                return Brushes.Green;
            }
            else if (IsRed(number))
            {
                return Brushes.Red;
            }
            else
            {
                return Brushes.Black;
            }
        }

        private bool IsRed(int number)
        {
            // European roulette red numbers
            int[] redNumbers = { 1,3,5,7,9,12,14,16,18,19,21,23,25,27,30,32,34,36 };
            return redNumbers.Contains(number);
        }

        private async void SpinButton_Click(object sender, RoutedEventArgs e)
        {
            // Disable buttons during spin
            SpinButton.IsEnabled = false;
            ResetButton.IsEnabled = false;

            MessageTextBlock.Text = "Spinning...";
            await SpinWheel();

            // Re-enable buttons after spin
            SpinButton.IsEnabled = true;
            ResetButton.IsEnabled = true;
        }

        private void ResetButton_Click(object sender, RoutedEventArgs e)
        {
            BallCanvas.Children.Clear();
            MessageTextBlock.Text = "";
        }

        private async Task SpinWheel()
        {
            // Randomly pick a winning number at the beginning
            int winningIndex = new Random().Next(SectorCount);
            int winningNumber = Numbers[winningIndex];

            // Calculate the final angle so that the winning number is at the top
            double finalWheelAngle = GetAngleForNumber(winningNumber);

            // Set total rotation to multiple of 360 plus finalWheelAngle
            int numberOfRotations = 5; // number of full rotations
            double totalRotation = 360 * numberOfRotations + finalWheelAngle;

            double duration = 5; // seconds
            double currentRotation = 0;
            DateTime startTime = DateTime.Now;

            RotateTransform wheelTransform = new RotateTransform();
            WheelCanvas.RenderTransform = wheelTransform;

            // Move the ball
            BallCanvas.Children.Clear();
            Ellipse ball = new Ellipse();
            ball.Width = BallRadius * 2;
            ball.Height = BallRadius * 2;
            ball.Fill = Brushes.White;
            Canvas.SetLeft(ball, WheelRadius - BallRadius);
            Canvas.SetTop(ball, 0);
            BallCanvas.Children.Add(ball);

            RotateTransform ballTransform = new RotateTransform();
            BallCanvas.RenderTransform = ballTransform;

            while (currentRotation < totalRotation)
            {
                double elapsed = (DateTime.Now - startTime).TotalSeconds;
                double t = elapsed / duration;
                if (t > 1) t = 1;

                currentRotation = totalRotation * t * t * (3 - 2 * t); // ease out

                wheelTransform.Angle = currentRotation % 360;
                ballTransform.Angle = -currentRotation % 360;

                await Task.Delay(16); // approximately 60 FPS
            }

            // Ensure the wheel stops at finalWheelAngle
            wheelTransform.Angle = finalWheelAngle;

            // Set the ball's final position (opposite to the wheel)
            ballTransform.Angle = -finalWheelAngle;

            MessageTextBlock.Text = $"The winning number is {winningNumber}!";
        }

        private double GetAngleForNumber(int number)
        {
            int index = Numbers.IndexOf(number);
            double anglePerSector = 360.0 / SectorCount;
            // Adjust the angle calculation to align the winning number at the top
            return (360 - (index * anglePerSector + anglePerSector / 2) + 90) % 360;
        }
    }
}