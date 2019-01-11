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
using Unity.Mathematics;
using UnityEngine;

namespace Boid.LeoECS.Sample3 {

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

#if ENABLE_IL2CPP
    [Unity.IL2CPP.CompilerServices.Il2CppSetOption (Unity.IL2CPP.CompilerServices.Option.NullChecks, false)]
    [Unity.IL2CPP.CompilerServices.Il2CppSetOption (Unity.IL2CPP.CompilerServices.Option.ArrayBoundsChecks, false)]
#endif
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
                neighborsFromEntity = BoidEntityData.neighbors, //.ToConcurrent (),
                //positionFromEntity = BoidEntityData.positions,
                entitiesCount = EntitiesCount
            };

            var wall = new WallJob {
                scale = WallScale,
                thresh = WallDistance,
                weight = WallWeight,
                positions = BoidEntityData.positions,
                accelerations = BoidEntityData.accelerations,
                _right = new float3 (1, 0, 0),
                _up = new float3 (0, 1, 0),
                _fwd = new float3 (0, 0, 1),
                _left = new float3 (-1, 0, 0),
                _down = new float3 (0, -1, 0),
                _back = new float3 (0, 0, -1)
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

            [ReadOnly] public NativeArray<float3> velocities;
            [ReadOnly] public NativeArray<float3> positions;
            [NativeDisableParallelForRestriction]
            public NativeArray<int> neighborsFromEntity;

            public void Execute (int index) {

                var entity = index * (entitiesCount + 1);
                var pos0 = positions[index];
                var fwd0 = math.normalize (velocities[index]);
                distThresh *= distThresh;
                float3 to;
                int count = 0;

                for (int i = 0; i < entitiesCount; ++i) {
                    var neighbor = i;
                    if (neighbor == index) continue;

                    var pos1 = positions[neighbor];
                    to.x = pos1.x - pos0.x;
                    to.y = pos1.y - pos0.y;
                    to.z = pos1.z - pos0.z;

                    if ((to.x * to.x + to.y * to.y + to.z * to.z) < distThresh) {
                        to = math.normalize (to);
                        var prod = math.dot (to, fwd0);
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
            [ReadOnly] public NativeArray<float3> positions;
            public NativeArray<float3> accelerations;

            public float3 _right;
            public float3 _up;
            public float3 _fwd;
            public float3 _left;
            public float3 _down;
            public float3 _back;

            public void Execute (int index) {
                var pos = positions[index];
                var value1 = GetAccelAgainstWall (-scale - pos.x, ref _right, thresh, weight);
                var value2 = GetAccelAgainstWall (-scale - pos.y, ref _up, thresh, weight);
                var value3 = GetAccelAgainstWall (-scale - pos.z, ref _fwd, thresh, weight);
                var value4 = GetAccelAgainstWall (+scale - pos.x, ref _left, thresh, weight);
                var value5 = GetAccelAgainstWall (+scale - pos.y, ref _down, thresh, weight);
                var value6 = GetAccelAgainstWall (+scale - pos.z, ref _back, thresh, weight);
                var accel = accelerations[index] + new float3 (
                    value1.x + value2.x + value3.x + value4.x + value5.x + value6.x,
                    value1.y + value2.y + value3.y + value4.y + value5.y + value6.y,
                    value1.z + value2.z + value3.z + value4.z + value5.z + value6.z);
                accelerations[index] = accel;
            }

            [MethodImpl (MethodImplOptions.AggressiveInlining)]
            float3 GetAccelAgainstWall (float dist, ref float3 dir, float thresh, float weight) {
                if (dist < thresh) {
                    return dir * (weight / math.abs (dist / thresh));
                }
                return float3.zero;
            }
        }

        [BurstCompile]
        public struct SeparationJob : IJobParallelFor {
            [ReadOnly] public float separationWeight;
            [ReadOnly] public int entitiesCount;
            [ReadOnly] public NativeArray<int> neighborsFromEntity;
            [ReadOnly] public NativeArray<float3> positions;
            public NativeArray<float3> accelerations;

            public void Execute (int index) {

                var entity = index * (entitiesCount + 1);
                int count = neighborsFromEntity[entity];
                if (count == 0) return;
                int i = 0;
                var pos0 = positions[index];
                var force = float3.zero;

                while (i < count) {
                    var pos1 = positions[neighborsFromEntity[entity + (++i)]];
                    force += math.normalize (pos0 - pos1);
                }
                force *= separationWeight / count;
                var accel = accelerations[index] + force;
                accelerations[index] = accel;
            }
        }

        [BurstCompile]
        public struct AlignmentJob : IJobParallelFor {
            [ReadOnly] public float alignmentWeight;
            [ReadOnly] public int entitiesCount;
            [ReadOnly] public NativeArray<int> neighborsFromEntity;
            [ReadOnly] public NativeArray<float3> velocities;
            public NativeArray<float3> accelerations;

            public void Execute (int index) {

                var entity = index * (entitiesCount + 1);
                int count = neighborsFromEntity[entity];
                if (count == 0) return;
                int i = 0;
                var averageVelocity = float3.zero;

                while (i < count) {
                    averageVelocity += velocities[neighborsFromEntity[entity + (++i)]];
                }
                averageVelocity /= count;
                var dAccel = (averageVelocity - velocities[index]) * alignmentWeight;
                var accel = dAccel + accelerations[index];
                accelerations[index] = accel;
            }
        }

        [BurstCompile]
        public struct CohesionJob : IJobParallelFor {
            [ReadOnly] public float cohesionWeight;
            [ReadOnly] public int entitiesCount;
            [ReadOnly] public NativeArray<int> neighborsFromEntity;
            [ReadOnly] public NativeArray<float3> positions;
            public NativeArray<float3> accelerations;

            public void Execute (int index) {

                var entity = index * (entitiesCount + 1);
                int count = neighborsFromEntity[entity];
                if (count == 0) return;
                int i = 0;
                var averagePos = float3.zero;

                while (i < count) {
                    averagePos += positions[neighborsFromEntity[entity + (++i)]];
                }
                averagePos /= count;
                var dAccel = (averagePos - positions[index]) * cohesionWeight;
                var accel = dAccel + accelerations[index];
                accelerations[index] = accel;
            }
        }

        [BurstCompile]
        public struct MoveJob : IJobParallelFor {
            [ReadOnly] public float dt;
            [ReadOnly] public float minSpeed;
            [ReadOnly] public float maxSpeed;
            [ReadOnly] public Vector3 scale;

            public NativeArray<float3> positions;
            //public NativeArray<float3> rotations;
            public NativeArray<float3> velocities;
            public NativeArray<float3> accelerations;
            public NativeArray<Matrix4x4> matrices;

            public void Execute (int index) {
                var v = velocities[index];
                var accel = accelerations[index];
                float3 velocity;
                velocity.x = v.x + accel.x * dt;
                velocity.y = v.y + accel.y * dt;
                velocity.z = v.z + accel.z * dt;
                var speed = math.length (velocity);
                var dir = velocity / speed;
                var vi = math.clamp (speed, minSpeed, maxSpeed) * dir;
                var pos = positions[index] + vi * dt;
                positions[index] = pos;
                accelerations[index] = float3.zero;
                velocities[index] = vi;
                var r = quaternion.LookRotationSafe (dir, Vector3.up);
                matrices[index] = Matrix4x4.TRS (pos, r, scale);
            }
        }
    }

#if ENABLE_IL2CPP
    [Unity.IL2CPP.CompilerServices.Il2CppSetOption (Unity.IL2CPP.CompilerServices.Option.NullChecks, false)]
    [Unity.IL2CPP.CompilerServices.Il2CppSetOption (Unity.IL2CPP.CompilerServices.Option.ArrayBoundsChecks, false)]
#endif
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