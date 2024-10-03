namespace FunnyRouletteCore.Physics
{
    /// <summary>
    /// Interface for physics objects in the simulation.
    /// </summary>
    public interface IPhysicsObject
    {
        void Update(double deltaTime);
    }
}