using System.Runtime.CompilerServices;
using Leopotam.Ecs;
using Unity.Burst;
using Unity.Collections;
using Unity.Entities;
using Unity.Jobs;
using Unity.Mathematics;
using UnityEngine;

namespace Boid.LeoECS.Sample3 {

    public sealed class BoidEntityData {

        public static NativeArray<float3> positions;
        public static NativeArray<float3> velocities;
        public static NativeArray<float3> accelerations;
        public static NativeArray<int> neighbors;

        public static void Dispose () {
            if (positions.IsCreated)
                positions.Dispose ();
            if (velocities.IsCreated)
                velocities.Dispose ();
            if (accelerations.IsCreated)
                accelerations.Dispose ();
            if (neighbors.IsCreated)
                neighbors.Dispose ();
        }
        public static void Create (int capacity, Allocator allocator = Allocator.Persistent) {
            Dispose ();
            positions = new NativeArray<float3> (capacity, allocator);
            velocities = new NativeArray<float3> (capacity, allocator);
            accelerations = new NativeArray<float3> (capacity, allocator);
            neighbors = new NativeArray<int> (capacity * (capacity + 1), allocator);
        }

        public int id;

        public float3 Position {
            get {
                return positions[id];
            }
            set {
                positions[id] = value;
            }
        }

        public float3 Velocity {
            get {
                return velocities[id];
            }
            set {
                velocities[id] = value;
            }
        }

        public float3 Acceleration {
            get {
                return accelerations[id];
            }
            set {
                accelerations[id] = value;
            }
        }
    }

    // public sealed class BoidEntityPositionsData {
    //     public static NativeArray<float3> positions;
    //     public int id;

    //     public float3 Position {
    //         get {
    //             return positions[id];
    //         }
    //         set {
    //             positions[id] = value;
    //         }
    //     }
    // }

    // public sealed class BoidEntityVelocitiesData {
    //     public static NativeArray<float3> velocities;
    //     public int id;

    //     public float3 Velocity {
    //         get {
    //             return velocities[id];
    //         }
    //         set {
    //             velocities[id] = value;
    //         }
    //     }
    // }

    // public sealed class BoidEntityAccelerationsData {
    //     public static NativeArray<float3> accelerations;
    //     public int id;

    //     public float3 Acceleration {
    //         get {
    //             return accelerations[id];
    //         }
    //         set {
    //             accelerations[id] = value;
    //         }
    //     }
    // }

    // public sealed class BoidEntityNeighborsData {
    //     public static NativeArray<int> neighbors;
    //     public int id;
    // }

    public class BoidEntityGraphics : IEcsAutoResetComponent {
        public Transform transform;

        [MethodImpl (MethodImplOptions.AggressiveInlining)]
        void IEcsAutoResetComponent.Reset () {
            transform = null;
        }
    }
}