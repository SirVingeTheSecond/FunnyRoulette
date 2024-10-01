namespace FunnyRouletteConsole.Physics
{
    /// <summary>
    /// Manages the physics simulation.
    /// </summary>
    public class PhysicsEngine
    {
        private readonly List<IPhysicsObject> _objects = new List<IPhysicsObject>();

        public void AddObject(IPhysicsObject obj)
        {
            _objects.Add(obj);
        }

        public void Update(double deltaTime)
        {
            foreach (var obj in _objects)
            {
                obj.Update(deltaTime);
            }
        }
    }
}