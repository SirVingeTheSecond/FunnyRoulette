namespace FunnyRouletteCore.Physics;

public class PhysicsLogger
{
    public enum LogLevel { Info, Debug, Warning }
    public static LogLevel CurrentLogLevel { get; set; } = LogLevel.Debug;

    public static void Log(string message, LogLevel level = LogLevel.Info)
    {
        if (level >= CurrentLogLevel)
            Console.WriteLine($"[{level}] {message}");
    }
}
