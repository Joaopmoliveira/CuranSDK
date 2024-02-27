# VSGParticleSystem
A simple particle system project based from the [VulkanSceneGraph](https://github.com/vsg-dev/VulkanSceneGraph) framework and [Niels Lohmann's JSON parser](https://github.com/nlohmann/json).

> A CMake, C++17 based project

This is a product for simulating various particles' trajectories, with each one of them representing a robotic arm's end-effector's planar movement, with a normal force (showing here by the vertical axis).

## Checkout repository
For HTTP:
```bash
    git clone https://github.com/filipe-varela/VSGParticleSystem.git
```

For ssh:
```bash
    git clone git@github.com:filipe-varela/VSGParticleSystem.git
```

## Unix and macOS build

    cd VSGParticleSystem
    cmake .
    make -j 8

## Windows build
From the VulkanSceneGraph:

> According to community feedback, it's working fine under Windows, but we're waiting on a Windows expert to volunteer an explanation, in the meantime have a look through the [VulkanSceneGraph/INSTALL.md](https://github.com/vsg-dev/VulkanSceneGraph/blob/master/INSTALL.md#detailed-instructions-for-setting-up-your-environment-and-building-for-microsoft-windows).

## Running the VSGParticleSystem program

You could run the executable without any argument, but here is a list of the specific arguments you could give:

```bash
bin/vsgparticlesystem --dyn <path/to/dynamics.json> -z <z_elevation (defaults to 0)> -n <number_of_particles (defaults to 5)>
```

Where it is also possible to provide the following flags:
- `--debug` or `-d` for debug purposes;
- `--api` or `-a` for dumping the API layer on the terminal;
- `--fullscreen` or `--fs` to run the program in fullscreen mode;
- `--random` or `-r` to make the Particles' initialization random in all directions, instead on a disk centered at `(0,0, z_elevation)`;
- `--free-center` or `--fc` to updates center through time;
- `--free-radius` or `--fr` to updates the axis radius through time;

## Dependencies tree
If you want to reuse some piece of the code from this project, here is a dependencies for the main modules in this project:

- **Axis:** 
    - [`ResolutionBuilder.h`](./src/ResolutionBuilder.h)
    - [`BuilderProps.h`](./src/BuilderProps.h)
    - [`Particle.h`](./src/Particle.h)
    - [`Text.h`](./src/Text.h)
    - [`vsgAdditionalFunctions.h`](./src/vsgAdditionalFunctions.h)
    - [`vsgAdditionalOperators.h`](./src/vsgAdditionalOperators.h)
- **Dynamics:** 
    - [`ResolutionBuilder.h`](./src/ResolutionBuilder.h)
    - [`BuilderProps.h`](./src/BuilderProps.h)
    - [`Particle.h`](./src/Particle.h)
    - [`mat5.h`](./src/mat5.h)
    - [`vec5.h`](./src/vec5.h)
    - [`Niels Lohmann's JSON parser`](https://github.com/nlohmann/json)
- **Particle:** 
    - [`ResolutionBuilder.h`](./src/ResolutionBuilder.h)
    - [`BuilderProps.h`](./src/BuilderProps.h)
- **ParticlesPool:** 
    - [`Axis.h`](./src/Axis.h)
    - [`Particle.h`](./src/Particle.h)
    - [`ParticleInitializer.h`](./src/ParticleInitializer.h)
