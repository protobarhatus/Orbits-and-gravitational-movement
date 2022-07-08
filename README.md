# orbits
This is a light project  using C++ with SFML. It provides an engine to simulate gravitational movement with few extra features about *transfer orbits* like
*Bi-elliptic transfer*, *Hohmann transfer orbit* or arbitrary double-pulse maneuver. There is two ready programms, but it also
can be used as a library.
It was written as a university task, 1 grade, MIPT.
## Simulation
Class `Space` provides a gravitational simulation in Newton's mechanic. It does that with arbitrary *integration method* of Runge–Kutta family that may be customized
by *ButchersTable*. By default the standart method of 6 order is set, but it can be changed by `setIntegrationMethod(const ButcherTable & method)` function.
To add objects, use `addPlanets` function.
By function `integrateStep(double step)`, one step of simulation is made - time is considered to move on `step` seconds and integration method will be executed with
`step` as a parameter. Usually, the lower is `step`, the more precisely it will be done. So, simulation can be made like that:
```cpp
Space space;

space.addPlanet(....);
/* add arbitraty amount of planets*/

space.prepareForSimulation();
while (true)
{
    space.integrateSpace(step_length);
    drawStage(space, window);
}
```

## ButcherTable
### About Butcher's table
Butcher's table is method of defining exact method of Runge-Kutta family. It looks like that:

![image](https://user-images.githubusercontent.com/51193423/177999674-a176f02d-5612-4b25-a352-47fc98da61cc.png)



In programm, it is represented by `ButcherTable` class. To create table, use constructor function
`ButchersTable(const Table& table, int amount_of_result_strings, bool is_explicit)`, where `table` is `std::vector<std::vector<long double>>` object with
all of coefficients placed exactly as in picture. The `amount_of_result_strings` parameter is responsible for amount of *b* lines under the table (used for some
fancy methods like Dorman-Prince, but for standart methods it must be 1). The is_explicit parameter is showing if method is explicit.
Explicit methods are methods with triangle table that looks like that:

![image](https://user-images.githubusercontent.com/51193423/177998989-9fbda891-280d-4a46-bf0f-b61221d4f174.png)

!For current moment, only explicit method can be integrated. Inexplicit method will give an error.

### Presets
There are some standart methods that can be received via functions:
```cpp
ButchersTable dormanPrince();
ButchersTable gaussLegendreFourthOrder();
ButchersTable tableForRK4();
ButchersTable LStable3rdOrderImplicitRungeKutta();
ButchersTable fifthOrderRadau();
ButchersTable tableForRK6();
ButchersTable explicitEuler();
```

where RK4 and Rk6 are classical methods of corresponding order.
### Integration of equations
Lets imagine we have equation `dy/dt = f(t, y)`, where y is a Vector and we have value y(t<sub>0</sub>). To get a value of y(t<sub>0</sub> + *step*), use
function:
```cpp
Vector ButcherRK(const ButchersTable& table, int result_string, Vector d, long double t, long double step, const std::function < Vector(long double, const Vector&)>& f,
	std::vector<std::pair<double, Vector>> * saved_values_at_proceeded_times = nullptr);
```
where `table` is integration method, `result_string` is what line of `b` params in ButcherTable will be used (almost always its 1), `d` is t<sub>0</sub>,
`f` is `f(t, y)`, `saved_values_at_proceeded_times` is optional parameter. If its not nullptr, then values of y(t<sub>0</sub> + _b1 * step_),
y(t<sub>0</sub> + _b1 * step + b2 * step_) etc will be saved there (object must be created and deleted externally).

### Integration of systems of equation.
Lets imagine we have system of equations:

![image](https://user-images.githubusercontent.com/51193423/178008728-d2451fc8-9419-474f-aeb1-c5fb7b18d11d.png)

and values y<sub>1</sub>(t), ... , y<sub>n</sub>(t) at given time *t*.
Then, use function
```cpp
std::vector<Vector> ButcherRkForSystem(const ButchersTable& table, const std::vector<Vector>& start_values, long double t, long double step,
	const std::vector<std::function<Vector(long double, const std::vector<Vector>& variables)>>& funcs_for_each_var);
```
where table is integration method, start_values is an array of y<sub>1</sub>(t), ... y<sub>n</sub>(t), funcs_for_each_var is an array of functions
f<sub>1</sub>(x, y<sub>1</sub>, ..., y<sub>n</sub>), ..., f<sub>n</sub>(x, y<sub>1</sub>, y<sub>n</sub>)

## Planets
Class `Object` represent objects that `Space` operates with. They represent planets, moons, satellites, etc. Each object has mass, position, and velocity.
When they are in Space, their movement is dictated by Newton's second law and law of gravity. Object may have a zero mass (for satellites) - it moves as
usual, as gravitational acceleration is not dependent from mass, but it don't influence other bodies. Also object may be set immovable - then it is not influenced
by any other body, but still creates gravitational field.

## Maneuvers
This section is created for the only situation - there is stationary massive Earth and satellite that orbits it. Then one can occasionally initiate gravitational
maneurs by these three functions:
```cpp
void executeHohmannManeuver(long double r2);
void executeBiellipticManeuver(long double r, long double b);

void executeDoublePulseManeuver(long double r2, long double dv_to_v);
```

description of maneuvers is given below.

## Orbital maneuver programm.
This is program with predefined stage: Earth is at the center of screen at satellite is orbiting it. One can control it by following set of commands:
1. HM *d*, where *d* is a positive rational number. Can be executed only if satellite is orbiting round orbit at a radius *r*. Then it will execute
a Hohmann transfer orbit and move to the round orbit with radius r * d by exactly two pulses.
2. BE *a* *b*, where *a*, *b* are positive rational numbers. Similar to HM, but Bi-elliptic maneuver will be executed - probe will make a pulse
to fly to distance _r*b_, then make a pulse to move to orbit with radius _r*a_ and stay on it with the third pulse.
3. DP *a* *b*, where *a*, is positive rational number and *b* is rational number bigger than -1. If *r* is probe's orbit radius (it must be round) and *v* is probe's velocity, then probe
will move to round orbit with radius _r * a_, with initial pulse that gives it velocity _v * (1 + b)_. Note that, for example, if *b* is positive and *a* < 1, then
maneuver will never be completed and, if *b* < 0.41, probe will fly at elliptic orbit from now on, and if b > 0.41, then it will fly to infinity as
its velocity will be bigger then escape velocity (вторая космическая).
4. width *w* where *w* is integer number - will set width of space fragment on screen
5. scale *s* where *s* is positive rational number - will change scale *s* times
6. vel or velocity - will print probe's current velocity
7. pulse *p* where *p* is integer number - will increase probe's velocity by _p*v_
8. restart - return program to initial state

### Example






https://user-images.githubusercontent.com/51193423/178021989-7d5c8705-6633-4f38-9223-d3ee48c61a4d.mp4

### Orbit
On video you can notice that sometimes appears and sometimes it's simple trail. It works like this: when probe passed certain amount of points, the orbit is
calculated based on these points. To be more precise, it happens when probe moves 150 degrees around Earth. It cannot be lower as then orbit will not be
precise enough. Every pulse will reset orbit and while it is not counted, there is just trail.
And yes, in this program orbit may be counted immediately by formulas - but i wanted to do it by points cause it is more fun and challenging.

### Purpose
Right now in this mode there is only Earth and satellite, since that's what i needed for physics. But under a little work it can be runned with more complex
solar system, simulating gravitational maneuvers between planets, for example.


## Simulation
Second program is straightforward simulation of gravitational movement of N bodies. Program receives input in following format:
```
<amount of bodies>
then N lines with description of each body in following format:
<mass> <x_position> <y_position> <x_velocity> <y_velocity> <color> <radius in pixels>
....
<space size> <Gravitational constant>
```
`Color` is one of these strings:
- white
- red
- green
- yellow
- magenta
- cyan
- orange
- blue
- light_blue

`radius in pixels` influences only drawing, in physics model, objects are point-like.

### Example
Here is example of simulation of inner Solar system


https://user-images.githubusercontent.com/51193423/178079293-6ffa19c9-06b0-4292-8ff0-a7dfd30a4858.mp4

(Moon is not in scale since in reallity its too close to earth and would be inside its sprite)

Text of it:
```
6
332900 0 0 0 0 yellow 10
0.055  -0.3 0 0 -0.0340176 red 3
0.815 0 -0.718 -0.0203644 0 orange 5
1 0.98 0 0 0.0174939 light_blue 5
0.0123 0.98 0.03876 0.000155382 0.0174939 white 3
0.107 0 1.381 -0.015 0 red 5
4 0.0000000008.88
```

## Installing and compiling.

This is a Visual Studio project, so create project from github in it. Install SFML from https://www.sfml-dev.org/index.php.
In `Source.cpp` find int main() {} function. To use orbit maneuver program, type there `orbits();` and compile. To use simulation programm, type `simulation();`
and compile






