## ECS.Examples.Boids
Simple Boids simulation example. Used for tests with PureECS, SveltoECS and LeoECS.

Based on
-----------
- https://github.com/hecomi/UnityECSBoidsSimulation
- https://github.com/sebas77/Svelto.ECS.Examples.Boids
- https://github.com/Leopotam/ecs
- https://github.com/Leopotam/ecs-threads
- https://github.com/Leopotam/ecs-types
-----------
# Timing results on android (Google Pixel):
![results1](https://user-images.githubusercontent.com/6071298/51064287-b997ab00-1620-11e9-9359-979041a60b31.png)
-----------
# Results
Kinda strange.
- SveltoECS with IL2CPP shows no boids at all.
- PureECS shows same results with mono and IL2CPP (probably because of that stupid bruteforce neighbors detection algorithm)
- LeoEcs+Jobs shows better results than ecs-threads, but I don't think it will be usable with more complex structure of components and systems

Environment
-----------
- Unity 2018.3

License
-------
<a rel="license" href="http://creativecommons.org/licenses/by/4.0/"><img alt="Creative Commons License" style="border-width:0" src="https://i.creativecommons.org/l/by/4.0/88x31.png" /></a><br />This work is licensed under a <a rel="license" href="http://creativecommons.org/licenses/by/4.0/">Creative Commons Attribution 4.0 International License</a>.
