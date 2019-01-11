﻿using Unity.Entities;
using Unity.Jobs;
using Unity.Transforms;
using Unity.Collections;
using Unity.Mathematics;
using UnityEngine;

namespace Boid.PureECS.Sample3
{

public class BoidsSystemGroup {}

[UpdateBefore(typeof(BoidsSystemGroup))]
public class NeighborDetectionSystem : JobComponentSystem
{
    public struct Job : IJobProcessComponentDataWithEntity<Position, Velocity>
    {
        [ReadOnly] public float prodThresh;
        [ReadOnly] public float distThresh;
        [ReadOnly] public ComponentDataFromEntity<Position> positionFromEntity;
        [ReadOnly] public BufferFromEntity<NeighborsEntityBuffer> neighborsFromEntity;
        [ReadOnly] public EntityArray entities;

        public void Execute(
            Entity entity,
            int index,
            [ReadOnly] ref Position pos,
            [ReadOnly] ref Velocity velocity)
        {
            neighborsFromEntity[entity].Clear();

            float3 pos0 = pos.Value;
            float3 fwd0 = math.normalize(velocity.Value);

            for (int i = 0; i < entities.Length; ++i)
            {
                var neighbor = entities[i];
                if (neighbor == entity) continue;

                float3 pos1 = positionFromEntity[neighbor].Value;
                var to = pos1 - pos0;
                var dist = math.length(to);

                if (dist < distThresh)
                {
                    var dir = math.normalize(to);
                    var prod = Vector3.Dot(dir, fwd0);
                    if (prod > prodThresh)
                    {
                        neighborsFromEntity[entity].Add(new NeighborsEntityBuffer { Value = neighbor });
                    }
                }
            }
        }
    }

    ComponentGroup group;

    protected override void OnCreateManager()
    {
        group = GetComponentGroup(typeof(Position), typeof(Velocity), typeof(NeighborsEntityBuffer));
    }

    protected override JobHandle OnUpdate(JobHandle inputDeps)
    {
        var job = new Job
        {
            prodThresh = math.cos(math.radians(Bootstrap.Param.neighborFov)),
            distThresh = Bootstrap.Param.neighborDistance,
            neighborsFromEntity = GetBufferFromEntity<NeighborsEntityBuffer>(false),
            positionFromEntity = GetComponentDataFromEntity<Position>(true),
            entities = group.GetEntityArray(),
        };
        return job.Schedule(this, inputDeps);
    }
}

[UpdateInGroup(typeof(BoidsSystemGroup))]
public class WallSystem : JobComponentSystem
{
    public struct Job : IJobProcessComponentData<Position, Acceleration>
    {
        [ReadOnly] public float scale;
        [ReadOnly] public float thresh;
        [ReadOnly] public float weight;

        public void Execute([ReadOnly] ref Position pos, ref Acceleration accel)
        {
            accel = new Acceleration
            {
                Value = accel.Value +
                    GetAccelAgainstWall(-scale - pos.Value.x, new float3(+1, 0, 0), thresh, weight) +
                    GetAccelAgainstWall(-scale - pos.Value.y, new float3(0, +1, 0), thresh, weight) +
                    GetAccelAgainstWall(-scale - pos.Value.z, new float3(0, 0, +1), thresh, weight) +
                    GetAccelAgainstWall(+scale - pos.Value.x, new float3(-1, 0, 0), thresh, weight) +
                    GetAccelAgainstWall(+scale - pos.Value.y, new float3(0, -1, 0), thresh, weight) +
                    GetAccelAgainstWall(+scale - pos.Value.z, new float3(0, 0, -1), thresh, weight)
            };
        }

        float3 GetAccelAgainstWall(float dist, float3 dir, float thresh, float weight)
        {
            if (dist < thresh)
            {
                return dir * (weight / math.abs(dist / thresh));
            }
            return float3.zero;
        }
    }

    protected override JobHandle OnUpdate(JobHandle inputDeps)
    {
        var job = new Job 
        {
            scale = Bootstrap.Param.wallScale * 0.5f,
            thresh = Bootstrap.Param.wallDistance,
            weight = Bootstrap.Param.wallWeight,
        };
        return job.Schedule(this, inputDeps);
    }
}

[UpdateInGroup(typeof(BoidsSystemGroup))]
public class SeparationSystem : JobComponentSystem
{
    public struct Job : IJobProcessComponentDataWithEntity<Position, Acceleration>
    {
        [ReadOnly] public float separationWeight;
        [ReadOnly] public BufferFromEntity<NeighborsEntityBuffer> neighborsFromEntity;
        [ReadOnly] public ComponentDataFromEntity<Position> positionFromEntity;

        public void Execute(Entity entity, int index, [ReadOnly] ref Position pos, ref Acceleration accel)
        {
            var neighbors = neighborsFromEntity[entity].Reinterpret<Entity>();
            if (neighbors.Length == 0) return;

            var pos0 = pos.Value;

            var force = float3.zero;
            for (int i = 0; i < neighbors.Length; ++i)
            {
                var pos1 = positionFromEntity[neighbors[i]].Value;
                force += math.normalize(pos0 - pos1);
            }
            force /= neighbors.Length;

            var dAccel = force * separationWeight;
            accel = new Acceleration { Value = accel.Value + dAccel };
        }
    }

    protected override JobHandle OnUpdate(JobHandle inputDeps)
    {
        var job = new Job 
        {
            separationWeight = Bootstrap.Param.separationWeight,
            neighborsFromEntity = GetBufferFromEntity<NeighborsEntityBuffer>(true),
            positionFromEntity = GetComponentDataFromEntity<Position>(true),
        };
        return job.Schedule(this, inputDeps);
    }
}

[UpdateInGroup(typeof(BoidsSystemGroup))]
public class AlignmentSystem : JobComponentSystem
{
    public struct Job : IJobProcessComponentDataWithEntity<Velocity, Acceleration>
    {
        [ReadOnly] public float alignmentWeight;
        [ReadOnly] public BufferFromEntity<NeighborsEntityBuffer> neighborsFromEntity;
        [ReadOnly] public ComponentDataFromEntity<Velocity> velocityFromEntity;

        public void Execute(Entity entity, int index, [ReadOnly] ref Velocity velocity, ref Acceleration accel)
        {
            var neighbors = neighborsFromEntity[entity].Reinterpret<Entity>();
            if (neighbors.Length == 0) return;

            var averageVelocity = float3.zero;
            for (int i = 0; i < neighbors.Length; ++i)
            {
                averageVelocity += velocityFromEntity[neighbors[i]].Value;
            }
            averageVelocity /= neighbors.Length;

            var dAccel = (averageVelocity - velocity.Value) * alignmentWeight;
            accel = new Acceleration { Value = accel.Value + dAccel };
        }
    }

    protected override JobHandle OnUpdate(JobHandle inputDeps)
    {
        var job = new Job 
        {
            alignmentWeight = Bootstrap.Param.alignmentWeight,
            neighborsFromEntity = GetBufferFromEntity<NeighborsEntityBuffer>(true),
            velocityFromEntity = GetComponentDataFromEntity<Velocity>(true),
        };
        return job.Schedule(this, inputDeps);
    }
}

[UpdateInGroup(typeof(BoidsSystemGroup))]
public class CohesionSystem : JobComponentSystem
{
    public struct Job : IJobProcessComponentDataWithEntity<Position, Acceleration>
    {
        [ReadOnly] public float cohesionWeight;
        [ReadOnly] public BufferFromEntity<NeighborsEntityBuffer> neighborsFromEntity;
        [ReadOnly] public ComponentDataFromEntity<Position> positionFromEntity;

        public void Execute(Entity entity, int index, [ReadOnly] ref Position pos, ref Acceleration accel)
        {
            var neighbors = neighborsFromEntity[entity].Reinterpret<Entity>();
            if (neighbors.Length == 0) return;

            var averagePos = float3.zero;
            for (int i = 0; i < neighbors.Length; ++i)
            {
                averagePos += positionFromEntity[neighbors[i]].Value;
            }
            averagePos /= neighbors.Length;

            var dAccel = (averagePos - pos.Value) * cohesionWeight;
            accel = new Acceleration { Value = accel.Value + dAccel };
        }
    }

    protected override JobHandle OnUpdate(JobHandle inputDeps)
    {
        var job = new Job 
        {
            cohesionWeight = Bootstrap.Param.cohesionWeight,
            neighborsFromEntity = GetBufferFromEntity<NeighborsEntityBuffer>(true),
            positionFromEntity = GetComponentDataFromEntity<Position>(true),
        };
        return job.Schedule(this, inputDeps);
    }
}

[UpdateAfter(typeof(BoidsSystemGroup))]
public class MoveSystem : JobComponentSystem
{
    public struct Job : IJobProcessComponentData<Position, Rotation, Velocity, Acceleration>
    {
        [ReadOnly] public float dt;
        [ReadOnly] public float minSpeed;
        [ReadOnly] public float maxSpeed;

        public void Execute(
            ref Position pos,
            [WriteOnly] ref Rotation rot,
            ref Velocity velocity,
            ref Acceleration accel)
        {
            var v = velocity.Value;
            v += accel.Value * dt;
            var dir = math.normalize(v);
            var speed = math.length(v);
            v = math.clamp(speed, minSpeed, maxSpeed) * dir;

            pos = new Position { Value = pos.Value + v * dt };
            rot = new Rotation { Value = quaternion.LookRotationSafe(dir, new float3(0, 1, 0)) };
            velocity = new Velocity { Value = v };
            accel = new Acceleration { Value = float3.zero };
        }
    }

    protected override JobHandle OnUpdate(JobHandle inputDeps)
    {
        var job = new Job
        {
            dt = Time.deltaTime,
            minSpeed = Bootstrap.Param.minSpeed,
            maxSpeed = Bootstrap.Param.maxSpeed,
        };
        return job.Schedule(this, inputDeps);
    }
}

}