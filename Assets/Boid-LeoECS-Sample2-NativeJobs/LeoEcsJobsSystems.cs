using System.Collections;
using System.Collections.Generic;
using System.Runtime.CompilerServices;
using Leopotam.Ecs;
using Leopotam.Ecs.Threads;
using Leopotam.Ecs.Types;
using Unity.Burst;
using Unity.Collections;
//using Unity.Entities;
using Unity.Jobs;
using UnityEngine;

namespace Boid.LeoECS.Sample2 {

    public sealed class TimeSystem : IEcsRunSystem {
        public float Time;
        public float DeltaTime;

        void IEcsRunSystem.Run () {
            Time = UnityEngine.Time.time;
            DeltaTime = UnityEngine.Time.deltaTime;
        }
    }

    [EcsInject]
    public sealed class BoidsSimulationSystem : IEcsRunSystem {
        EcsWorld _world;

        void IEcsRunSystem.Run () {

        }
    }

    [EcsInject]
    public sealed class JobMoveSystem : IEcsInitSystem, IEcsRunSystem {
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

        public static int EntitiesCount, NumBoidsPerJob;
        public static Vector3 BoidScale;
        public static bool CanRead = true;
        public static bool FullBackground = false;
        EcsWorld _world = null;
        EcsFilter<BoidEntityData> _filter = null;
        TimeSystem _timeSystem = null;
        JobHandle jobHandle;

        void IEcsInitSystem.Initialize () { }

        void IEcsInitSystem.Destroy () {
            CanRead = false;
            jobHandle.Complete ();
        }

        void IEcsRunSystem.Run () {

            if (_filter.EntitiesCount == 0)
                return;

            if (FullBackground) {
                if (!CanRead) {
                    jobHandle.Complete ();
                    CanRead = true;
                }
            }

            var neighbors = new NeighborsDetectionJob {
                prodThresh = NeighborFov,
                distThresh = NeighborDistance,
                velocities = BoidEntityData.velocities,
                positions = BoidEntityData.positions,
                neighborsFromEntity = BoidEntityData.neighbors,
                entitiesCount = EntitiesCount
            };

            var wall = new WallJob {
                scale = WallScale,
                thresh = WallDistance,
                weight = WallWeight,
                positions = BoidEntityData.positions,
                accelerations = BoidEntityData.accelerations,
                _right = new Float3 (1, 0, 0),
                _up = new Float3 (0, 1, 0),
                _fwd = new Float3 (0, 0, 1),
                _left = new Float3 (-1, 0, 0),
                _down = new Float3 (0, -1, 0),
                _back = new Float3 (0, 0, -1)
            };

            var separation = new SeparationJob {
                separationWeight = SeparationWeight,
                entitiesCount = EntitiesCount,
                neighborsFromEntity = BoidEntityData.neighbors,
                positions = BoidEntityData.positions,
                accelerations = BoidEntityData.accelerations
            };

            var alignment = new AlignmentJob {
                alignmentWeight = AlignmentWeight,
                entitiesCount = EntitiesCount,
                neighborsFromEntity = BoidEntityData.neighbors,
                velocities = BoidEntityData.velocities,
                accelerations = BoidEntityData.accelerations
            };

            var cohesion = new CohesionJob {
                cohesionWeight = CohesionWeight,
                entitiesCount = EntitiesCount,
                neighborsFromEntity = BoidEntityData.neighbors,
                positions = BoidEntityData.positions,
                accelerations = BoidEntityData.accelerations
            };

            var move = new MoveJob {
                dt = _timeSystem.DeltaTime,
                minSpeed = MinSpeed,
                maxSpeed = MaxSpeed,
                scale = BoidScale,
                positions = BoidEntityData.positions,
                velocities = BoidEntityData.velocities,
                accelerations = BoidEntityData.accelerations,
                matrices = BoidsVisualisationSystem._nativeMatrices
            };

            jobHandle = neighbors.Schedule (EntitiesCount, NumBoidsPerJob);
            jobHandle = wall.Schedule (EntitiesCount, NumBoidsPerJob, jobHandle);
            jobHandle = separation.Schedule (EntitiesCount, NumBoidsPerJob, jobHandle);
            jobHandle = alignment.Schedule (EntitiesCount, NumBoidsPerJob, jobHandle);
            jobHandle = cohesion.Schedule (EntitiesCount, NumBoidsPerJob, jobHandle);
            jobHandle = move.Schedule (EntitiesCount, NumBoidsPerJob, jobHandle);
            JobHandle.ScheduleBatchedJobs ();
            if (FullBackground) {
                CanRead = false;
            } else {
                jobHandle.Complete ();
                CanRead = true;
            }
        }

        [BurstCompile]
        public struct NeighborsDetectionJob : IJobParallelFor {
            [ReadOnly] public float prodThresh;
            [ReadOnly] public float distThresh;
            [ReadOnly] public int entitiesCount;

            [ReadOnly] public NativeArray<Float3> velocities;
            [ReadOnly] public NativeArray<Float3> positions;
            [NativeDisableParallelForRestriction]
            public NativeArray<int> neighborsFromEntity;

            public void Execute (int index) {

                var entity = index * (entitiesCount + 1);
                var pos0 = positions[index];
                var vel0 = velocities[index];
                var fwd0 = Float3.Normalize (ref vel0);
                distThresh *= distThresh;
                Float3 to;
                int count = 0;

                for (int i = 0; i < entitiesCount; ++i) {
                    var neighbor = i;
                    if (neighbor == index) continue;

                    var pos1 = positions[neighbor];
                    to.X = pos1.X - pos0.X;
                    to.Y = pos1.Y - pos0.Y;
                    to.Z = pos1.Z - pos0.Z;

                    if ((to.X * to.X + to.Y * to.Y + to.Z * to.Z) < distThresh) {
                        to.Normalize ();
                        var prod = Float3.Dot (ref to, ref fwd0);
                        if (prod > prodThresh) {
                            neighborsFromEntity[entity + (++count)] = neighbor;
                        }
                    }
                }
                neighborsFromEntity[entity] = count;
            }
        }

        [BurstCompile]
        public struct WallJob : IJobParallelFor {
            [ReadOnly] public float scale;
            [ReadOnly] public float thresh;
            [ReadOnly] public float weight;
            [ReadOnly] public NativeArray<Float3> positions;
            public NativeArray<Float3> accelerations;

            public Float3 _right;
            public Float3 _up;
            public Float3 _fwd;
            public Float3 _left;
            public Float3 _down;
            public Float3 _back;

            public void Execute (int index) {
                var pos = positions[index];
                var value1 = GetAccelAgainstWall (-scale - pos.X, ref _right, thresh, weight);
                var value2 = GetAccelAgainstWall (-scale - pos.Y, ref _up, thresh, weight);
                var value3 = GetAccelAgainstWall (-scale - pos.Z, ref _fwd, thresh, weight);
                var value4 = GetAccelAgainstWall (+scale - pos.X, ref _left, thresh, weight);
                var value5 = GetAccelAgainstWall (+scale - pos.Y, ref _down, thresh, weight);
                var value6 = GetAccelAgainstWall (+scale - pos.Z, ref _back, thresh, weight);
                var accel = accelerations[index];
                accel.Add (
                    value1.X + value2.X + value3.X + value4.X + value5.X + value6.X,
                    value1.Y + value2.Y + value3.Y + value4.Y + value5.Y + value6.Y,
                    value1.Z + value2.Z + value3.Z + value4.Z + value5.Z + value6.Z);
                accelerations[index] = accel;
            }

            [MethodImpl (MethodImplOptions.AggressiveInlining)]
            Float3 GetAccelAgainstWall (float dist, ref Float3 dir, float thresh, float weight) {
                if (dist < thresh) {
                    return Float3.Scale (ref dir, weight / MathFast.Abs (dist / thresh));
                }
                return Float3.Zero;
            }
        }

        [BurstCompile]
        public struct SeparationJob : IJobParallelFor {
            [ReadOnly] public float separationWeight;
            [ReadOnly] public int entitiesCount;
            [ReadOnly] public NativeArray<int> neighborsFromEntity;
            [ReadOnly] public NativeArray<Float3> positions;
            public NativeArray<Float3> accelerations;

            public void Execute (int index) {

                var entity = index * (entitiesCount + 1);
                int count = neighborsFromEntity[entity];
                if (count == 0) return;
                int i = 0;
                var pos0 = positions[index];
                var accel = accelerations[index];
                var force = Float3.Zero;
                Float3 n;

                while (i < count) {
                    var pos1 = positions[neighborsFromEntity[entity + (++i)]];
                    n = Float3.Sub (ref pos0, ref pos1);
                    n.Normalize ();
                    force.Add (ref n);
                }

                force.Scale (separationWeight / count);
                accel.Add (ref force);
                accelerations[index] = accel;

            }
        }

        [BurstCompile]
        public struct AlignmentJob : IJobParallelFor {
            [ReadOnly] public float alignmentWeight;
            [ReadOnly] public int entitiesCount;
            [ReadOnly] public NativeArray<int> neighborsFromEntity;
            [ReadOnly] public NativeArray<Float3> velocities;
            public NativeArray<Float3> accelerations;

            public void Execute (int index) {

                var entity = index * (entitiesCount + 1);
                int count = neighborsFromEntity[entity];
                if (count == 0) return;
                int i = 0;
                var averageVelocity = Float3.Zero;
                var entityVelocity = velocities[index];

                while (i < count) {
                    var vel = velocities[neighborsFromEntity[entity + (++i)]];
                    averageVelocity.Add (ref vel);
                }

                averageVelocity.Scale (1f / count);
                averageVelocity.Sub (ref entityVelocity);
                averageVelocity.Scale (alignmentWeight);
                var accel = accelerations[index];
                accel.Add (ref averageVelocity);
                accelerations[index] = accel;
            }
        }

        [BurstCompile]
        public struct CohesionJob : IJobParallelFor {
            [ReadOnly] public float cohesionWeight;
            [ReadOnly] public int entitiesCount;
            [ReadOnly] public NativeArray<int> neighborsFromEntity;
            [ReadOnly] public NativeArray<Float3> positions;
            public NativeArray<Float3> accelerations;

            public void Execute (int index) {

                var entity = index * (entitiesCount + 1);
                int count = neighborsFromEntity[entity];
                if (count == 0) return;
                int i = 0;
                var averagePos = Float3.Zero;
                var pos = positions[index];

                while (i < count) {
                    var npos = positions[neighborsFromEntity[entity + (++i)]];
                    averagePos.Add (ref npos);
                }
                averagePos.Scale (1f / count);
                averagePos = Float3.Sub (ref averagePos, ref pos);
                averagePos.Scale (cohesionWeight);
                var accel = accelerations[index];
                accel.Add (ref averagePos);
                accelerations[index] = accel;
            }
        }

        [BurstCompile]
        public struct MoveJob : IJobParallelFor {
            [ReadOnly] public float dt;
            [ReadOnly] public float minSpeed;
            [ReadOnly] public float maxSpeed;
            [ReadOnly] public Vector3 scale;

            public NativeArray<Float3> positions;
            //public NativeArray<Float3> rotations;
            public NativeArray<Float3> velocities;
            public NativeArray<Float3> accelerations;
            public NativeArray<Matrix4x4> matrices;

            public void Execute (int index) {
                var v = velocities[index];
                var accel = accelerations[index];
                Float3 velocity;
                velocity.X = v.X + accel.X * dt;
                velocity.Y = v.Y + accel.Y * dt;
                velocity.Z = v.Z + accel.Z * dt;
                var speed = Float3.Magnitude (ref velocity);
                velocity.Scale (1f / speed);
                var r = Quaternion.LookRotation (velocity, Vector3.up);
                speed = MathFast.Clamp (speed, minSpeed, maxSpeed);
                velocity.Scale (speed);
                var pos = positions[index];
                pos.Add (velocity.X * dt, velocity.Y * dt, velocity.Z * dt);
                positions[index] = pos;
                accel.Set (0f, 0f, 0f);
                accelerations[index] = accel;
                velocities[index] = velocity;

                matrices[index] = Matrix4x4.TRS (pos, r, scale);
            }
        }
    }

    [EcsInject]
    public sealed class BoidsVisualisationSystem : IEcsInitSystem, IEcsRunSystem {
        public Mesh mesh;
        public Material material;
        public Vector3 scale;

        EcsWorld _world = null;
        EcsFilter<BoidEntityData> _filter = null;
        public static NativeArray<Matrix4x4> _nativeMatrices;
        public static Matrix4x4[] _matrices = new Matrix4x4[0];

        void IEcsInitSystem.Initialize () { }

        void IEcsInitSystem.Destroy () { }

        void IEcsRunSystem.Run () {
            if (JobMoveSystem.CanRead)
                _nativeMatrices.CopyTo (_matrices);
            Graphics.DrawMeshInstanced (mesh, 0, material, _matrices, _filter.EntitiesCount);
        }
    }
}