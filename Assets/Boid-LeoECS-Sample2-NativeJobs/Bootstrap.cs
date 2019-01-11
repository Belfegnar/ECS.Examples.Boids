using Leopotam.Ecs;
using Leopotam.Ecs.Types;
using Unity.Collections;
using Unity.Jobs;
using UnityEngine;

namespace Boid.LeoECS.Sample2 {
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

                BoidEntityData.Dispose ();
                BoidsVisualisationSystem._nativeMatrices.Dispose ();
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
            JobMoveSystem.MinSpeed = param.minSpeed;
            JobMoveSystem.MaxSpeed = param.maxSpeed;
            JobMoveSystem.BoidScale = boidScale;
            JobMoveSystem.WallScale = param.wallScale * 0.5f;
            JobMoveSystem.WallDistance = param.wallDistance;
            JobMoveSystem.WallWeight = param.wallWeight;
            JobMoveSystem.NeighborFov = MathFast.Cos (param.neighborFov * MathFast.Deg2Rad);
            JobMoveSystem.NeighborDistance = param.neighborDistance;
            JobMoveSystem.SeparationWeight = param.separationWeight;
            JobMoveSystem.AlignmentWeight = param.alignmentWeight;
            JobMoveSystem.CohesionWeight = param.cohesionWeight;
            JobMoveSystem.EntitiesCount = boidCount;
            JobMoveSystem.NumBoidsPerJob = boidCount / System.Environment.ProcessorCount;

            if (BoidsVisualisationSystem._nativeMatrices.IsCreated)
                BoidsVisualisationSystem._nativeMatrices.Dispose ();
            BoidsVisualisationSystem._nativeMatrices = new NativeArray<Matrix4x4> (boidCount, Allocator.Persistent);
            BoidsVisualisationSystem._matrices = new Matrix4x4[boidCount];

            var timeSystem = new TimeSystem ();
            _systems = new EcsSystems (_world);
            _systems
                .Add (timeSystem)
                .Add (new BoidsSimulationSystem ())
                .Add (new JobMoveSystem ())
                .Add (new BoidsVisualisationSystem () { mesh = mesh, material = material, scale = boidScale })
                .Inject (timeSystem)
                .Initialize ();
            var random = new RngXorShift (853);

            BoidEntityData.Create (boidCount);

            for (int i = 0; i < boidCount; ++i) {
                var initSpeed = param.initSpeed;
                //init them with these default values
                var boid = _world.CreateEntityWith<BoidEntityData> ();
                boid.id = i;
                boid.Position = new Float3 (random.GetFloat (), random.GetFloat (), random.GetFloat ());
                var vel = new Float3 (random.GetFloat (), random.GetFloat (), random.GetFloat ());
                vel.Normalize ();
                vel.Scale (initSpeed);
                boid.Velocity = vel;
                boid.Acceleration = Float3.Zero;
            }
            _world.ProcessDelayedUpdates ();
        }

        void OnDrawGizmos () {
            if (!param) return;
            Gizmos.color = Color.green;
            Gizmos.DrawWireCube (Vector3.zero, Vector3.one * param.wallScale);
        }

        // void OnGUI () {
        //     if (GUILayout.Button (string.Format ("FullBackground is {0}", JobMoveSystem.FullBackground))) {
        //         JobMoveSystem.FullBackground = !JobMoveSystem.FullBackground;
        //     }
        // }

        EcsWorld _world;
        EcsSystems _systems;
    }
}