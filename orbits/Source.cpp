#include <Graphics.hpp>
#include "MyLib.h"
#include "Space.h"
#include <chrono>
#include "Orbit.h"
#include <iostream>
#include <string>
#include <thread>
#include <assert.h>
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

int main()
{

	sf::ContextSettings settings;
	settings.antialiasingLevel = 8;

	int screen_size = 1000;
	sf::RenderWindow window(sf::VideoMode(screen_size, screen_size), "Plot", sf::Style::Default, settings);

	

	Space space;
	space.addPlanet(getEarth(Vector(0, 0)));
	space.setProbe(Object(1, Vector(0, -100), Vector(3.16227766, 0), Vector(0, 0)));
	//space.setProbe(Object(1, Vector(0, -200), Vector(1, 0), Vector(0, 0)));
	//space.setProbe(Object(1, Vector(0, 50), Vector(4, 0), Vector(0, 0)));
	std::chrono::time_point<std::chrono::steady_clock> t1, start;
	std::chrono::nanoseconds last_dif(0);
	std::chrono::nanoseconds period = 1ms;

	std::thread commands(recieveCommands);
	commands.detach();

	//int space_size = 6000e3;
	int space_size = 600;



	//Orbit orbit;
	//orbit.setOrbit(5, 2, 1, -1e6, -1e6, -1e12);
	while (true)
	{
		window.clear();
		start = std::chrono::steady_clock::now();
		//DrImage canvas(1000, 1000);
		//canvas.setWindow(&window);

		

		Vector probe_screen = fromSpaceToScreen(space.getProbe().getPosition(), space_size, screen_size);
		//canvas.drawCircle(probe_screen.x(), probe_screen.y(), 10, sf::Color::Color(255, 255, 0));
		drawCircle(probe_screen.x(), probe_screen.y(), 5, sf::Color::Color(255, 255, 0), &window);
		for (auto& it : space.getPlanets())
		{
			Vector pos = fromSpaceToScreen(it.getPosition(), space_size, screen_size);
			//canvas.drawCircle(pos.x(), pos.y(), 30, sf::Color::Blue);
			drawCircle(pos.x(), pos.y(), 30, sf::Color::Blue, &window);
		}
		
		//canvas.draw(0, 0);

		//drawCircumference(500, 500, 1000 / 3, 3, sf::Color::Red, &window);

		
		drawOrbit(space.getProbesOrbit(), space_size, screen_size, sf::Color::Red, &window);
		//if (space.getProbesOrbit().defined())
		//	drawCircle(toSf(fromSpaceToScreen(space.getProbesOrbit().gravitationalCentre(), space_size, screen_size)), 5, sf::Color::Magenta, &window);

		std::stringstream probepos;
		probepos << "R = " << round(length(space.getProbe().getPosition()));
		HdrawString(probepos.str(), 0, 0, 20, &window, sf::Color::Red);

		window.display();
		
		space.integrateStep(0.1);
		//if (equals(t, 250))
			//space.getProbePtr()->executeHohmannManeur(2e6);
			//space.getProbePtr()->executeHohmannManeur(200);
		static int c = 0;
		++c;
		//if (c % 20 == 0)
		//	std::cout << "Energy: " << sqr(length(space.getProbe().getVelocity())) / 2 - (1000.0 / length(space.getProbe().getPosition())) - space.getProbe().getPosition().x() * 0.0001 << std::endl;
		if (isOrbitManeurCommand(request_type))
		{

				
				switch (request_type)
				{
				case HOHMANN:
					space.getProbePtr()->executeHohmannManeur(request_args_double[0] * length(space.getProbe().getPosition()));
					break;
				case BI_ELLIPTIC:
					space.getProbePtr()->executeBiellipticManeur(request_args_double[0], request_args_double[1]);
					break;
				case DOUBLE_PULSE:
					space.getProbePtr()->executeDoublePulseManeur(request_args_double[0], request_args_double[1]);
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

		//std::cout << (std::chrono::steady_clock::now() - start).count() << std::endl;
		while (std::chrono::steady_clock::now() - start < period)
			;
	}

	
	return 0;
}