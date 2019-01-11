using System;
using Leopotam.Ecs;
using Leopotam.Ecs.Types;
using Unity.Collections;
//using Unity.Jobs;
using UnityEngine;

#if ENABLE_IL2CPP
// Unity IL2CPP performance optimization attribute.
namespace Unity.IL2CPP.CompilerServices {
    enum Option {
        NullChecks = 1,
        ArrayBoundsChecks = 2
    }

    [AttributeUsage (AttributeTargets.Class | AttributeTargets.Method | AttributeTargets.Property, Inherited = false, AllowMultiple = true)]
    class Il2CppSetOptionAttribute : Attribute {
        public Option Option { get; private set; }
        public object Value { get; private set; }

        public Il2CppSetOptionAttribute (Option option, object value) { Option = option; Value = value; }
    }
}
#endif

namespace Boid.LeoECS.Sample1 {
#if ENABLE_IL2CPP
    [Unity.IL2CPP.CompilerServices.Il2CppSetOption (Unity.IL2CPP.CompilerServices.Option.NullChecks, false)]
    [Unity.IL2CPP.CompilerServices.Il2CppSetOption (Unity.IL2CPP.CompilerServices.Option.ArrayBoundsChecks, false)]
#endif
    public class Bootstrap : MonoBehaviour {
        [SerializeField]
        public int boidCount = 100;

        [SerializeField]
        Vector3 boidScale = new Vector3 (0.1f, 0.1f, 0.3f);

        [SerializeField]
        Param param;

        [SerializeField]
        Mesh mesh;

        [SerializeField]
        Material material;

        void OnDestroy () {
            if (_systems != null) {
                // destroy systems logical group.
                _systems.Dispose ();
                _systems = null;
                // destroy world.
                _world.Dispose ();
                _world = null;
            }
        }

        void Start () {
            QualitySettings.vSyncCount = -1;
            Application.targetFrameRate = 1000;

            LeoECS ();
        }

        void Update () {
            // process all dependent systems.
            if (_systems != null) {
                _systems.Run ();
            }
        }

        void LeoECS () {
            _world = new EcsWorld ();
            ThreadMoveSystem.MinSpeed = param.minSpeed;
            ThreadMoveSystem.MaxSpeed = param.maxSpeed;
            ThreadMoveSystem.BoidScale = boidScale;
            ThreadMoveSystem.WallScale = param.wallScale * 0.5f;
            ThreadMoveSystem.WallDistance = param.wallDistance;
            ThreadMoveSystem.WallWeight = param.wallWeight;
            ThreadMoveSystem.NeighborFov = MathFast.Cos (param.neighborFov * MathFast.Deg2Rad);
            ThreadMoveSystem.NeighborDistance = param.neighborDistance;
            ThreadMoveSystem.SeparationWeight = param.separationWeight;
            ThreadMoveSystem.AlignmentWeight = param.alignmentWeight;
            ThreadMoveSystem.CohesionWeight = param.cohesionWeight;
            ThreadMoveSystem.Neighbours = new BoidEntityData[boidCount][];
            for (int i = 0; i < boidCount; i++) {
                ThreadMoveSystem.Neighbours[i] = new BoidEntityData[boidCount];
            }

            BoidsVisualisationSystem2._matrices = new Matrix4x4[boidCount];

            var timeSystem = new TimeSystem ();
            _systems = new EcsSystems (_world);
            _systems
                .Add (timeSystem)
                .Add (new BoidsSimulationSystem ())
                .Add (new ThreadMoveSystem ())
                .Add (new BoidsVisualisationSystem2 () { mesh = mesh, material = material, scale = boidScale })
                // .Add (new BoidsVisualisationSystem () { mesh = mesh, material = material, scale = boidScale })
                .Inject (timeSystem)
                .Initialize ();
            var random = new RngXorShift (853);

            for (int i = 0; i < boidCount; ++i) {
                var initSpeed = param.initSpeed;
                //init them with these default values
                var boid = _world.CreateEntityWith<BoidEntityData> ();
                boid.position = new Float3 (random.GetFloat (), random.GetFloat (), random.GetFloat ());
                boid.velocity = new Float3 (random.GetFloat (), random.GetFloat (), random.GetFloat ());
                boid.velocity.Normalize ();
                boid.velocity.Scale (initSpeed);
                boid.acceleration = Float3.Zero;
            }
            _world.ProcessDelayedUpdates ();
        }

        void OnDrawGizmos () {
            if (!param) return;
            Gizmos.color = Color.green;
            Gizmos.DrawWireCube (Vector3.zero, Vector3.one * param.wallScale);
        }

        EcsWorld _world;
        EcsSystems _systems;
    }
}