#include <Graphics.hpp>
#include "MyLib/MyLib.h"
#include "Space.h"
#include <chrono>
#include "Orbit.h"
#include <iostream>
#include <string>
#include <thread>
#include <assert.h>
#include "ProbeTrackingSpace.h"
using namespace std::chrono;

Vector fromSpaceToScreen(const Vector& pos, long double space_size, int screen_size)
{
	Vector refl(pos.x(), -pos.y());
	Vector scale = refl * (double(screen_size) / space_size);
	return scale + Vector(screen_size / 2, screen_size / 2);
}
#include <iostream>
#include <cassert>

sf::Vector2f toSf(const Vector& v)
{
	return sf::Vector2f(v.x(), v.y());
}

void drawCircumference(int x, int y, int rad, int width, sf::Color color, sf::RenderTarget* target)
{
	sf::CircleShape shape(rad);
	shape.setFillColor(sf::Color::Transparent);

	shape.setOutlineThickness(width);
	shape.setOutlineColor(color);

	shape.setPosition(sf::Vector2f(x - rad, y - rad));
	shape.setPointCount(4 * rad);
	target->draw(shape);
}


void drawCircle(int x, int y, int rad, sf::Color color, sf::RenderTarget* target)
{
	sf::CircleShape shape(rad);
	shape.setFillColor(color);

	shape.setPosition(sf::Vector2f(x - rad, y - rad));
	shape.setPointCount(4 * rad);
	target->draw(shape);
}

void drawCircle(const sf::Vector2f & v, int rad, sf::Color color, sf::RenderTarget* target)
{
	drawCircle(v.x, v.y, rad, color, target);
}

void HdrawString(std::string str, int X, int Y, int fontsize_in_pixels, sf::RenderTarget* window, sf::Color font_color, std::string font = "Microsoft Sans Serif.ttf")
{

	sf::Font ffont;
	if (!ffont.loadFromFile(font))
	{
		throw ("Error with font load, check file " + font);
	}
	sf::Text text;
	text.setFont(ffont);
	text.setString(str);
	text.setCharacterSize(fontsize_in_pixels);
	text.setFillColor(font_color);
	text.setPosition(sf::Vector2f(X, Y));
	window->draw(text);
}

void drawOrbit(Orbit& orbit, long double space_size, int screen_size, sf::Color color, sf::RenderTarget* target)
{
	/*sf::ConvexShape shape;
	shape.setPointCount(orbit.trajectoryPoints().size());
	for (int i = 0; i < orbit.trajectoryPoints().size(); ++i)
	{
		shape.setPoint(i, toSf(fromSpaceToScreen(orbit.trajectoryPoints()[i], space_size, screen_size)));
	}
	shape.setOutlineColor(color);
	shape.setOutlineThickness(3);
	shape.setFillColor(sf::Color::Transparent);
	target->draw(shape);*/
	if (orbit.inTraceMode() && !orbit.isNeedToRedraw())
	{
		if (orbit.lastVertex().size() == 0)
			return;
		orbit.addToGraphArray(sf::Vertex(toSf(fromSpaceToScreen(orbit.lastVertex(), space_size, screen_size)), color));
		target->draw(orbit.graphArray());
		return;
	}
	if (orbit.hasChanged() || orbit.isNeedToRedraw())
	{
		sf::VertexArray arr(sf::LineStrip, orbit.trajectoryPoints().size() + (orbit.inTraceMode() ? 0 : 1));

		for (int i = 0; i < orbit.trajectoryPoints().size(); ++i)
		{
			arr[i] = sf::Vertex(toSf(fromSpaceToScreen(orbit.trajectoryPoints()[i], space_size, screen_size)), color);
		}
		if (!orbit.inTraceMode())
			arr[orbit.trajectoryPoints().size()] = arr[0];

		target->draw(arr);

		orbit.setGraphArray(std::move(arr));
	}
	else
	{
		target->draw(orbit.graphArray());
	}


}
//это конечно не хорошо глобальные переменные и т д ноооо счас не до норм архитектуры
enum UsersRequest
{
	NONE,
	HOHMANN,
	BI_ELLIPTIC,
	DOUBLE_PULSE,
	SET_WIDTH,
	MULT_WIDTH,
	VELOCITY,
	PULSE,
	RESTART
};
bool isOrbitManeurCommand(UsersRequest command)
{
	return command == HOHMANN || command == BI_ELLIPTIC || command == DOUBLE_PULSE;
}

UsersRequest request_type;
long double request_args_double[5];
int request_args_int[5];

void recieveCommands()
{

	while (true)
	{
		std::string request;
		std::cin >> request;
		if (request == "HM")
		{
			
			std::cin >> request_args_double[0];
			request_type = HOHMANN;
		}
		else if (request == "BE")
		{
			
			std::cin >> request_args_double[0] >> request_args_double[1];
			request_type = BI_ELLIPTIC;
		}
		else if (request == "DP")
		{
			
			std::cin >> request_args_double[0] >> request_args_double[1];
			request_type = DOUBLE_PULSE;
		}
		else if (request == "width")
		{
			
			std::cin >> request_args_int[0];
			request_type = SET_WIDTH;
		}
		else if (request == "scale")
		{
			
			std::cin >> request_args_double[0];
			request_type = MULT_WIDTH;
		}
		else if (request == "vel" || request == "velocity")
		{
			request_type = VELOCITY;
		}
		else if (request == "pulse")
		{
			std::cin >> request_args_double[0];
			request_type = PULSE;
		}
		else if (request == "restart")
		{
			request_type = RESTART;
		}
		else
			std::cout << "Wrong request type\n";
	}
}

static long double sqr(long double a)
{
	return a * a;
}

void executeCommand(ProbeTrackingSpace& space, int& space_size)
{
	if (isOrbitManeurCommand(request_type))
	{
		switch (request_type)
		{
		case HOHMANN:
			space.getProbePtr()->executeHohmannManeuver(request_args_double[0] * length(space.getProbe().getPosition()));
			break;
		case BI_ELLIPTIC:
			space.getProbePtr()->executeBiellipticManeuver(request_args_double[0], request_args_double[1]);
			break;
		case DOUBLE_PULSE:
			space.getProbePtr()->executeDoublePulseManeuver(request_args_double[0], request_args_double[1]);
			break;
		default:
			assert(false);

		}
		request_type = NONE;
	}
	else if (request_type != NONE)
	{
		switch (request_type)
		{
		case SET_WIDTH:
			space_size = request_args_int[0];
			space.getProbesOrbit().needToRedraw();
			break;
		case MULT_WIDTH:
			space_size /= request_args_double[0];
			space.getProbesOrbit().needToRedraw();
			break;
		case VELOCITY:
			std::cout << "Velocity: " << round(length(space.getProbePtr()->getVelocity()) * 100) / 100 << "\n\n";
			break;
		case PULSE:
			space.getProbePtr()->setVelocity(space.getProbePtr()->getVelocity() + space.getProbePtr()->getVelocity() / length(space.getProbePtr()->getVelocity()) * request_args_double[0]);
			space.getProbesOrbit().orbitChanged();
			break;
		case RESTART:
			space.getProbePtr()->setPosition(Vector(0, -100));
			space.getProbePtr()->setVelocity(Vector(3.16227766, 0));
			space.getProbesOrbit().orbitChanged();
			break;
		default:
			assert(false);
		}
		request_type = NONE;
	}
}

void drawOrbitsScene(ProbeTrackingSpace& space, sf::RenderWindow& window, int screen_size, long double space_size)
{
	window.clear();
	Vector probe_screen = fromSpaceToScreen(space.getProbe().getPosition(), space_size, screen_size);
	drawCircle(probe_screen.x(), probe_screen.y(), 5, sf::Color::Color(255, 255, 0), &window);
	for (int i = 0; i < space.getAmountOfObjects(); ++i)
	{
		if (i == space.getProbeIndex())
			continue;
		Vector pos = fromSpaceToScreen(space.getPlanets()[i].getPosition(), space_size, screen_size);
		drawCircle(pos.x(), pos.y(), 30, sf::Color::Blue, &window);
	}
	drawOrbit(space.getProbesOrbit(), space_size, screen_size, sf::Color::Red, &window);

	std::stringstream probepos;
	probepos << "R = " << round(length(space.getProbe().getPosition()));
	HdrawString(probepos.str(), 0, 0, 20, &window, sf::Color::Red);

	window.display();
}

void orbits()
{
	sf::ContextSettings settings;
	settings.antialiasingLevel = 8;

	int screen_size = 1000;
	sf::RenderWindow window(sf::VideoMode(screen_size, screen_size), "Plot", sf::Style::Default, settings);



	ProbeTrackingSpace space;
	space.addPlanet(getEarth(Vector(0, 0)));
	space.setProbe(Object(0.0001, Vector(0, -100), Vector(3.16227766, 0), sf::Color::Yellow, 5));
	space.prepareForSimulation();

	std::chrono::time_point<std::chrono::steady_clock> t1, start;
	std::chrono::nanoseconds last_dif(0);
	std::chrono::nanoseconds period = 33ms;

	std::thread commands(recieveCommands);
	commands.detach();
	//its size of space fragment that fits to the screen
	long double space_size = 600;
	drawOrbitsScene(space, window, screen_size, space_size);
	start = std::chrono::steady_clock::now();
	while (true)
	{
		space.integrateStep(0.1);
		
		if (std::chrono::steady_clock::now() - start >= period)
			drawOrbitsScene(space, window, screen_size, space_size);
	}


	
}

void drawScene(const Space& space, sf::RenderWindow& window, int screen_size, long double space_size)
{
	window.clear();
	for (int i = 0; i < space.getAmountOfObjects(); ++i)
	{
		Vector obj_pos = space.getObjectPosition(i);
		Vector pos = fromSpaceToScreen(obj_pos, space_size, screen_size);
		drawCircle(pos.x(), pos.y(), space.getObjectWithoutUpdatingParams(i).getRadius(), space.getObjectWithoutUpdatingParams(i).getColor(), &window);
	}
	window.display();
}

sf::Color strToColor(const std::string& str)
{
	if (str == "white")
		return sf::Color::White;
	if (str == "red")
		return sf::Color::Red;
	if (str == "green")
		return sf::Color::Green;
	if (str == "yellow")
		return sf::Color::Yellow;
	if (str == "magenta")
		return sf::Color::Magenta;
	if (str == "cyan")
		return sf::Color::Cyan;
	if (str == "orange")
		return sf::Color(255, 106, 0);
	if (str == "blue")
		return sf::Color(0, 0, 255);
	if (str == "light_blue")
		return sf::Color(22, 135, 255);
	std::cout << "wrong color: blue used\n";
	return sf::Color(0, 0, 255);
}
Object readPlanet()
{
	long double mass, xpos, ypos, xvel, yvel;
	std::string color;
	int rad;
	std::cin >> mass >> xpos >> ypos >> xvel >> yvel >> color >> rad;
	return Object(mass, Vector(xpos, ypos), Vector(xvel, yvel), strToColor(color), rad, true);
}

void simulation()
{
	sf::ContextSettings settings;
	settings.antialiasingLevel = 8;

	int screen_size = 1000;
	sf::RenderWindow window(sf::VideoMode(screen_size, screen_size), "Plot", sf::Style::Default, settings);

	std::chrono::time_point<std::chrono::steady_clock> t1, start;
	std::chrono::nanoseconds last_dif(0);
	std::chrono::nanoseconds period = 33ms;

	//its size of space fragment that fits to the screen
	long double space_size = 5000;

	Space space;


	int amount_of_planets;
	std::cin >> amount_of_planets;

	for (int i = 0; i < amount_of_planets; ++i)
		space.addPlanet(readPlanet());

	long double G;
	std::cin >> space_size >> G;
	std::cout << "Launching simulation!\n";

	space.setGravitational(G);
	space.prepareForSimulation();

	drawScene(space, window, screen_size, space_size);
	start = std::chrono::steady_clock::now();
	while (true)
	{
		space.integrateStep(0.1);

		if (std::chrono::steady_clock::now() - start >= period)
			drawScene(space, window, screen_size, space_size);
	}
}

int main()
{
	simulation();
	
}