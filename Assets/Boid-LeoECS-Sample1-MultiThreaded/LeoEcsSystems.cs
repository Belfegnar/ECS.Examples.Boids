using System.Collections;
using System.Collections.Generic;
using System.Runtime.CompilerServices;
using Leopotam.Ecs;
using Leopotam.Ecs.Threads;
using Leopotam.Ecs.Types;
using UnityEngine;

namespace Boid.LeoECS.Sample1 {
#if ENABLE_IL2CPP
    [Unity.IL2CPP.CompilerServices.Il2CppSetOption (Unity.IL2CPP.CompilerServices.Option.NullChecks, false)]
    [Unity.IL2CPP.CompilerServices.Il2CppSetOption (Unity.IL2CPP.CompilerServices.Option.ArrayBoundsChecks, false)]
#endif
    public sealed class TimeSystem : IEcsRunSystem {
        public float Time;
        public float DeltaTime;

        void IEcsRunSystem.Run () {
            Time = UnityEngine.Time.time;
            DeltaTime = UnityEngine.Time.deltaTime;
        }
    }

#if ENABLE_IL2CPP
    [Unity.IL2CPP.CompilerServices.Il2CppSetOption (Unity.IL2CPP.CompilerServices.Option.NullChecks, false)]
    [Unity.IL2CPP.CompilerServices.Il2CppSetOption (Unity.IL2CPP.CompilerServices.Option.ArrayBoundsChecks, false)]
#endif
    [EcsInject]
    public sealed class BoidsSimulationSystem : IEcsRunSystem {
        EcsWorld _world;

        void IEcsRunSystem.Run () {

        }
    }

#if ENABLE_IL2CPP
    [Unity.IL2CPP.CompilerServices.Il2CppSetOption (Unity.IL2CPP.CompilerServices.Option.NullChecks, false)]
    [Unity.IL2CPP.CompilerServices.Il2CppSetOption (Unity.IL2CPP.CompilerServices.Option.ArrayBoundsChecks, false)]
#endif
    [EcsInject]
    public sealed class ThreadMoveSystem : EcsMultiThreadSystem<EcsFilter<BoidEntityData>> {
        public static float MinSpeed,
        MaxSpeed,
        WallScale,
        WallDistance,
        WallWeight,
        NeighborFov,
        NeighborDistance,
        SeparationWeight,
        AlignmentWeight,
        CohesionWeight;

        public static BoidEntityData[][] Neighbours;

        public static Vector3 BoidScale;
        EcsWorld _world = null;
        EcsFilter<BoidEntityData> _filter = null;
        TimeSystem _timeSystem = null;

        /// <summary>
        /// Returns filter for processing entities in it at background threads.
        /// </summary>
        protected override EcsFilter<BoidEntityData> GetFilter () {
            return _filter;
        }

        /// <summary>
        /// Returns minimal amount of entities for splitting to threads instead processing in one.
        /// </summary>
        protected override int GetMinJobSize () {
            return 50;
        }

        /// <summary>
        /// Returns background threads amount. Main thread will be used as additional worker (+1 thread).
        /// </summary>
        protected override int GetThreadsCount () {
            return System.Environment.ProcessorCount - 1;
        }

        /// <summary>
        /// Returns our worker callback.
        /// </summary>
        protected override EcsMultiThreadWorker GetWorker () {
            return Worker;
        }

        /// <summary>
        /// Our worker callback for processing entities.
        /// Important: always use static methods as workers - you cant touch any instance data except method parameters.
        /// </summary>
        void Worker (EcsFilter<BoidEntityData> filter, int from, int to) {
            // Important: you can't iterate over filter with foreach-loop
            // and should use for-loop with "from" / "to" bounds.
            var dt = _timeSystem.DeltaTime;
            var minSpeed = MinSpeed;
            var maxSpeed = MaxSpeed;
            var scale = BoidScale;

            var wallScale = WallScale;
            var thresh = WallDistance;
            var weight = WallWeight;

            var prodThresh = NeighborFov;
            var distThresh = NeighborDistance;
            var separationWeight = SeparationWeight;
            var alignmentWeight = AlignmentWeight;
            var cohesionWeight = CohesionWeight;

            Float3 velocity = Float3.Zero;
            Vector3 up = Vector3.up;

            for (var i = from; i < to; i++) {
                var c = filter.Components1[i];
                Wall (c, wallScale, thresh, weight);
                var neighbours = Neighbours[i];
                var currentCount = DetectNeighbour (c, distThresh, prodThresh, filter.Components1, filter.EntitiesCount, neighbours);
                if (currentCount > 0) {
                    Separation (c, separationWeight, currentCount, neighbours);
                    Alignment (c, alignmentWeight, currentCount, neighbours);
                    Cohesion (c, cohesionWeight, currentCount, neighbours);
                }
                velocity.X = c.velocity.X + c.acceleration.X * dt;
                velocity.Y = c.velocity.Y + c.acceleration.Y * dt;
                velocity.Z = c.velocity.Z + c.acceleration.Z * dt;
                var speed = Float3.Magnitude (ref velocity);
                velocity.Scale (1f / speed);
                var r = Quaternion.LookRotation (velocity, up);
                speed = MathFast.Clamp (speed, minSpeed, maxSpeed);
                velocity.Scale (speed);
                c.position.Add (velocity.X * dt, velocity.Y * dt, velocity.Z * dt);
                c.acceleration.Set (0f, 0f, 0f);
                c.velocity = velocity;

                BoidsVisualisationSystem2._matrices[i] = Matrix4x4.TRS (c.position, r, scale);
            }
        }

        [MethodImpl (MethodImplOptions.AggressiveInlining)]
        static int DetectNeighbour (BoidEntityData entity, float distThresh, float prodThresh, BoidEntityData[] entities, int count, BoidEntityData[] neighbours) {
            int currentCount = 0;
            var pos0 = entity.position;
            var fwd0 = Float3.Normalize (ref entity.velocity);
            distThresh *= distThresh;
            Float3 to;
            for (int i = 0, iMax = count; i < iMax; i++) {
                var pos1 = entities[i].position;
                to.X = pos1.X - pos0.X;
                to.Y = pos1.Y - pos0.Y;
                to.Z = pos1.Z - pos0.Z;
                if ((to.X * to.X + to.Y * to.Y + to.Z * to.Z) < distThresh) {
                    to.Normalize ();
                    var prod = Float3.Dot (ref to, ref fwd0);
                    if (prod > prodThresh) {
                        neighbours[currentCount++] = entities[i];
                    }
                }
            }
            return currentCount;
        }

        [MethodImpl (MethodImplOptions.AggressiveInlining)]
        static void Separation (BoidEntityData entity, float separationWeight, int count, BoidEntityData[] neighbours) {
            var pos0 = entity.position;
            var force = Float3.Zero;
            Float3 n;
            for (int i = 0; i < count; ++i) {
                n = Float3.Sub (ref pos0, ref neighbours[i].position);
                n.Normalize ();
                force.Add (ref n);
            }
            force.Scale (separationWeight / count);
            entity.acceleration.Add (ref force);
        }

        [MethodImpl (MethodImplOptions.AggressiveInlining)]
        static void Alignment (BoidEntityData entity, float alignmentWeight, int count, BoidEntityData[] neighbours) {
            var averageVelocity = Float3.Zero;
            for (int i = 0; i < count; ++i) {
                averageVelocity.Add (ref neighbours[i].velocity);
            }
            averageVelocity.Scale (1f / count);
            averageVelocity.Sub (ref entity.velocity);
            averageVelocity.Scale (alignmentWeight);
            entity.acceleration.Add (ref averageVelocity);
        }

        [MethodImpl (MethodImplOptions.AggressiveInlining)]
        static void Cohesion (BoidEntityData entity, float cohesionWeight, int count, BoidEntityData[] neighbours) {
            var averagePos = Float3.Zero;
            for (var i = 0; i < count; ++i) {
                averagePos.Add (ref neighbours[i].position);
            }
            averagePos.Scale (1f / count);
            averagePos = Float3.Sub (ref averagePos, ref entity.position);
            averagePos.Scale (cohesionWeight);
            entity.acceleration.Add (ref averagePos);
        }

        static Float3 _right = new Float3 (1, 0, 0);
        static Float3 _up = new Float3 (0, 1, 0);
        static Float3 _fwd = new Float3 (0, 0, 1);
        static Float3 _left = new Float3 (-1, 0, 0);
        static Float3 _down = new Float3 (0, -1, 0);
        static Float3 _back = new Float3 (0, 0, -1);

        [MethodImpl (MethodImplOptions.AggressiveInlining)]
        static void Wall (BoidEntityData entity, float scale, float thresh, float weight) {
            var value1 = GetAccelAgainstWall (-scale - entity.position.X, ref _right, thresh, weight);
            var value2 = GetAccelAgainstWall (-scale - entity.position.Y, ref _up, thresh, weight);
            var value3 = GetAccelAgainstWall (-scale - entity.position.Z, ref _fwd, thresh, weight);
            var value4 = GetAccelAgainstWall (+scale - entity.position.X, ref _left, thresh, weight);
            var value5 = GetAccelAgainstWall (+scale - entity.position.Y, ref _down, thresh, weight);
            var value6 = GetAccelAgainstWall (+scale - entity.position.Z, ref _back, thresh, weight);
            entity.acceleration.Add (
                value1.X + value2.X + value3.X + value4.X + value5.X + value6.X,
                value1.Y + value2.Y + value3.Y + value4.Y + value5.Y + value6.Y,
                value1.Z + value2.Z + value3.Z + value4.Z + value5.Z + value6.Z);
        }

        [MethodImpl (MethodImplOptions.AggressiveInlining)]
        static Float3 GetAccelAgainstWall (float dist, ref Float3 dir, float thresh, float weight) {
            if (dist < thresh) {
                return Float3.Scale (ref dir, weight / MathFast.Abs (dist / thresh));
            }
            return Float3.Zero;
        }
    }

#if ENABLE_IL2CPP
    [Unity.IL2CPP.CompilerServices.Il2CppSetOption (Unity.IL2CPP.CompilerServices.Option.NullChecks, false)]
    [Unity.IL2CPP.CompilerServices.Il2CppSetOption (Unity.IL2CPP.CompilerServices.Option.ArrayBoundsChecks, false)]
#endif
    [EcsInject]
    public sealed class BoidsVisualisationSystem2 : IEcsInitSystem, IEcsRunSystem {
        public Mesh mesh;
        public Material material;
        public Vector3 scale;

        EcsWorld _world = null;
        EcsFilter<BoidEntityData> _filter = null;
        public static Matrix4x4[] _matrices = new Matrix4x4[0];

        void IEcsInitSystem.Initialize () { }

        void IEcsInitSystem.Destroy () { }

        void IEcsRunSystem.Run () {
            Graphics.DrawMeshInstanced (mesh, 0, material, _matrices, _filter.EntitiesCount);
        }
    }
}