#pragma once
#include <SFML\Graphics.hpp>
class DrImage
{
public:
	DrImage();
	DrImage(unsigned int width, unsigned int height, sf::Color default_color = sf::Color(0, 0, 0));
	DrImage(sf::Image && image);
	void setScalingAntialiasingCoefficient(double coe);
	void create(unsigned int width, unsigned int height, sf::Color default_color = sf::Color(0, 0, 0));
	~DrImage();
	void setWindow(sf::RenderWindow * window);
	void draw(int x, int y);
	void draw(int x, int y, sf::RenderWindow * window);
	//������� �������� ������ �������� � scale ���. ����� �������� �� ����� ����� draw scaled
	void drawReduced(unsigned int x, unsigned int y, double scale, sf::RenderWindow * window);
	void drawReduced(unsigned int x, unsigned int y, double scale, sf::RenderTarget * canvas);
	void setLinesAntialiasing(bool enabled);
	void drawLine(unsigned int x1, unsigned int y1, unsigned int x2, unsigned int y2, sf::Color color);
	void drawLine(sf::Vector2i a, sf::Vector2i b, sf::Color color);
	//��� ���������� �������, � ����� ������ �� �����. �������� ������ � ������������� �� ������ sf::Color::Black � sf::Color::White
	void blackWhiteAntialiasing();

	unsigned int getWidth() const;
	unsigned int getHeight() const;
	void saveToFile(std::string name);
	void draw(int x, int y, sf::RenderTarget * canvas);
	void draw(int x, int y, sf::RenderTexture * texture);
	sf::RenderWindow * getWindow();
	void drawCircle(int x, int y, int rad, sf::Color color);
	//shit
	void drawCircumference(int x, int y, int rad, int width, sf::Color color);
	void drawSquare(int x_left, int y_top, int width, int height, sf::Color color);
	bool setPixel(int x, int y, sf::Color color);
	bool inImage(int x, int y);
	
private:
	double antia_coe = 1.;
	//lines antialisasing
	bool is_antialiasing_enabled;
	bool is_scaling_antialiasing_enabled;
	void drawVertikalLine(unsigned int x, unsigned int y1, unsigned int y2, sf::Color color);
	void drawHorizontalLine(unsigned int x1, unsigned int x2, unsigned int y, sf::Color color);
	void drawNotAntialiasedLine(unsigned int x1, unsigned int y1, unsigned int x2, unsigned int y2, sf::Color color);
	void drawAntialiasedLine(unsigned int x1, unsigned int y1, unsigned int x2, unsigned int y2, sf::Color color);
	void algorithmVuOnXAxis(unsigned int x1, unsigned int y1, unsigned int x2, unsigned int y2, sf::Color color);
	void algorithmVuOnYAxis(unsigned int x1, unsigned int y1, unsigned int x2, unsigned int y2, sf::Color color);
	void drawAntialiasedBisectrix(unsigned int x1, unsigned int y1, unsigned int x2, unsigned int y2, sf::Color color);
	void update();
	//����� ���� ���������� � ��������� ������������ (��� � ������) �� ���������� �����
	sf::Texture texture;
	sf::RectangleShape sprite;
	sf::Image image;
	sf::RenderWindow * window;
	bool need_to_change_texture;
};

