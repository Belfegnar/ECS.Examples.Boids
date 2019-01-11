using System.Runtime.CompilerServices;
using Leopotam.Ecs;
using Leopotam.Ecs.Types;
//using Unity.Burst;
//using Unity.Collections;
//using Unity.Entities;
//using Unity.Jobs;
using UnityEngine;

namespace Boid.LeoECS.Sample1 {
#if ENABLE_IL2CPP
    [Unity.IL2CPP.CompilerServices.Il2CppSetOption (Unity.IL2CPP.CompilerServices.Option.NullChecks, false)]
    [Unity.IL2CPP.CompilerServices.Il2CppSetOption (Unity.IL2CPP.CompilerServices.Option.ArrayBoundsChecks, false)]
#endif
    public sealed class BoidEntityData {
        public Float3 position;
        public Float3 velocity;
        public Float3 acceleration;
    }

#if ENABLE_IL2CPP
    [Unity.IL2CPP.CompilerServices.Il2CppSetOption (Unity.IL2CPP.CompilerServices.Option.NullChecks, false)]
    [Unity.IL2CPP.CompilerServices.Il2CppSetOption (Unity.IL2CPP.CompilerServices.Option.ArrayBoundsChecks, false)]
#endif
    public class BoidEntityGraphics : IEcsAutoResetComponent {
        public Transform transform;

        [MethodImpl (MethodImplOptions.AggressiveInlining)]
        void IEcsAutoResetComponent.Reset () {
            transform = null;
        }
    }
}